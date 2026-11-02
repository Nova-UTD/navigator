"""Package:   helpers
Filename:  gpu.py
Author:    Ron Shaked
Email:     ronshaked07@gmail.com
Copyright: 2021, Nova UTD
License:   MIT License

Select the GPU with the most free memory and add node context to failures.
Memory readings are snapshots; selection does not reserve memory for a model.
Free memory is read through NVML so that probing never creates a CUDA context:
a context costs hundreds of MiB per GPU and fails outright on a full GPU.
"""

from collections.abc import Callable
from typing import TypeVar

import pynvml
import torch

Model = TypeVar("Model")

# Errors meaning this GPU cannot hold the model right now (full, or no room to
# create a CUDA context), so the next GPU is worth trying.
_GPU_FULL_ERRORS = ("out of memory", "busy or unavailable", "cuda_error_unknown")


def _free_bytes(index: int) -> int:
    """! Return free memory in bytes for torch device `index` without a CUDA context.

    Torch indices follow CUDA_VISIBLE_DEVICES and CUDA_DEVICE_ORDER while NVML
    indices are physical, so the device is matched by UUID.

    @param index[int] Torch CUDA device index.
    @return int Free memory in bytes.
    @throws RuntimeError If NVML cannot read the device.
    """
    try:
        pynvml.nvmlInit()
        uuid = f"GPU-{torch.cuda.get_device_properties(index).uuid}"
        handle = pynvml.nvmlDeviceGetHandleByUUID(uuid.encode())
        return pynvml.nvmlDeviceGetMemoryInfo(handle).free
    except pynvml.NVMLError as error:
        raise RuntimeError(f"NVML error: {error}") from error


def cuda_devices_by_free_memory(
    *,
    caller: str = "unspecified caller",
    purpose: str = "model initialization",
    min_free_memory_mb: int | None = None,
) -> list[torch.device]:
    """! Return visible GPUs meeting the minimum, most free memory first.

    Skip GPUs whose memory cannot be queried. Raise a descriptive error if
    CUDA cannot initialize, no GPUs are visible, or no readable GPU meets the
    optional minimum. This does not estimate model size or guarantee loading.

    @param caller[str] Node requesting a GPU, included in error messages.
    @param purpose[str] Model or operation that needs the GPU.
    @param min_free_memory_mb[int|None] Minimum free MiB; None means no minimum.
    @return list[torch.device] Candidate CUDA devices, best first.
    @throws ValueError If the minimum is negative.
    @throws RuntimeError If no suitable GPU can be selected.
    """
    if min_free_memory_mb is not None and min_free_memory_mb < 0:
        raise ValueError("min_free_memory_mb must be non-negative")

    context = f"{caller} cannot start {purpose}"
    try:
        # Initialize explicitly so driver failures retain their original message.
        # This loads the driver only; no per-GPU context is created.
        torch.cuda.init()
        device_count = torch.cuda.device_count()
    except (RuntimeError, AssertionError) as error:
        raise RuntimeError(f"{context}: CUDA initialization failed: {error}") from error

    if device_count == 0:
        raise RuntimeError(
            f"{context}: no CUDA GPUs are visible. Check GPU access and "
            "CUDA_VISIBLE_DEVICES in this environment."
        )

    readable: list[tuple[int, int]] = []
    report: list[str] = []
    for index in range(device_count):
        try:
            free_bytes = _free_bytes(index)
        except RuntimeError as error:
            report.append(f"cuda:{index}: memory query failed: {error}")
            continue
        report.append(f"cuda:{index}: {free_bytes / 1024**2:.0f} MiB free")
        readable.append((free_bytes, index))

    if not readable:
        raise RuntimeError(
            f"{context}: all GPU memory queries failed. {'; '.join(report)}"
        )
    minimum = (min_free_memory_mb or 0) * 1024**2
    candidates = [
        torch.device(f"cuda:{index}")
        for free_bytes, index in sorted(readable, key=lambda item: (-item[0], item[1]))
        if free_bytes >= minimum
    ]
    if not candidates:
        raise RuntimeError(
            f"{context}: no GPU with readable memory has the required "
            f"{min_free_memory_mb} MiB free. {'; '.join(report)}"
        )
    return candidates


def select_cuda_device(
    *,
    caller: str = "unspecified caller",
    purpose: str = "model initialization",
    min_free_memory_mb: int | None = None,
) -> torch.device:
    """! Return the visible GPU with the most free memory.

    See cuda_devices_by_free_memory for selection and error behavior.

    @return torch.device Selected CUDA device.
    """
    return cuda_devices_by_free_memory(
        caller=caller, purpose=purpose, min_free_memory_mb=min_free_memory_mb
    )[0]


def load_model_on_best_gpu(
    initializer: Callable[[torch.device], Model],
    *,
    caller: str,
    purpose: str,
    min_free_memory_mb: int | None = None,
    log: Callable[[str], None] = print,
) -> tuple[Model, torch.device]:
    """! Load a model on the GPU with the most free memory, falling back when full.

    Free memory can change between the query and the load (another process may
    allocate first), so an out-of-memory or context-creation failure moves on to
    the next candidate. Any other failure is raised immediately.

    @param initializer[Callable] Function that loads the model onto the device.
    @param caller[str] Node loading the model.
    @param purpose[str] Description of the loading operation.
    @param min_free_memory_mb[int|None] Minimum free MiB; None means no minimum.
    @param log[Callable] Receives a message before each load attempt.
    @return tuple[Model, torch.device] Loaded model and the device it is on.
    @throws RuntimeError If no GPU qualifies or every candidate runs out of memory.
    """
    candidates = cuda_devices_by_free_memory(
        caller=caller, purpose=purpose, min_free_memory_mb=min_free_memory_mb
    )
    failures: list[str] = []
    for device in candidates:
        log(f"{caller}: {purpose} on {device}")
        try:
            return (
                run_model_initializer(
                    lambda: initializer(device),
                    device=device,
                    caller=caller,
                    purpose=purpose,
                ),
                device,
            )
        except RuntimeError as error:
            if not any(text in str(error).lower() for text in _GPU_FULL_ERRORS):
                raise
            failures.append(f"{device}: {str(error).splitlines()[-1]}")
            torch.cuda.empty_cache()
    raise RuntimeError(
        f"{caller} failed while {purpose}: every candidate GPU was full or "
        f"unavailable. {'; '.join(failures)}"
    )


def run_model_initializer(
    initializer: Callable[[], Model],
    *,
    device: torch.device,
    caller: str,
    purpose: str,
) -> Model:
    """! Load the model and add node and device context to runtime failures.

    Call the supplied loader once and return its result. Preserve the original
    error (including CUDA out-of-memory or driver errors) as the cause. This
    does not check memory, move the model, or retry on another GPU.

    @param initializer[Callable] Function that loads the model onto device.
    @param device[torch.device] Device used by the loader.
    @param caller[str] Node loading the model.
    @param purpose[str] Description of the loading operation.
    @return Model Result returned by the loader.
    @throws RuntimeError Loading failure with node, device, and original error.
    """
    try:
        return initializer()
    except RuntimeError as error:
        raise RuntimeError(
            f"{caller} failed while {purpose} on {device}: {error}"
        ) from error


if __name__ == "__main__":
    print(select_cuda_device(caller="gpu.py demo"))

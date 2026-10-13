"""Package:   helpers
Filename:  gpu.py
Author:    Ron Shaked
Email:     ronshaked07@gmail.com
Copyright: 2021, Nova UTD
License:   MIT License

Provides shared GPU selection and model-initialization error handling for
inference nodes. ``select_cuda_device`` chooses the CUDA device with the most
free memory that meets the caller's optional minimum-memory threshold, or
returns CPU when CUDA is unavailable. ``run_model_initializer`` runs a supplied
model-loading function on the device chosen by the caller and adds node, task,
device, and memory details to CUDA out-of-memory errors. These checks do not
reserve GPU memory or guarantee that model allocation will succeed.
"""

from collections.abc import Callable

import torch


class CudaDeviceSelectionError(RuntimeError):
    """! Reports that a requested model cannot be assigned or loaded on CUDA.

    This error identifies GPU selection or model initialization failures with
    context supplied by the calling node. It is raised when CUDA memory cannot
    be queried, no GPU meets a caller's free-memory threshold, or a model
    initializer runs out of CUDA memory.
    """


def _free_memory_report(device_count: int) -> str:
    """! Build a readable snapshot of free and total memory on CUDA devices.

    Queries each device independently so one failed memory query does not
    prevent reporting the remaining GPUs. A query failure is included in the
    returned text instead of being raised from this diagnostic helper.

    @param device_count[int] Number of CUDA devices to query.
    @return str Comma-separated per-device memory details, or a message if no devices were reported.
    """
    entries = []
    for index in range(device_count):
        try:
            free_bytes, total_bytes = torch.cuda.mem_get_info(index)
            entries.append(
                f"cuda:{index}={free_bytes / 1024**2:.0f} MiB free"
                f"/{total_bytes / 1024**2:.0f} MiB total"
            )
        except Exception as error:  # CUDA can disappear or fail during a query.
            entries.append(f"cuda:{index}=memory query failed ({error})")
    return ", ".join(entries) or "no CUDA devices reported"


def select_cuda_device(
    *,
    caller: str = "unspecified caller",
    purpose: str = "model initialization",
    min_free_memory_mb: int | None = None,
) -> torch.device:
    """! Choose which device an inference node should use for its model.

    Checks the free memory on every visible CUDA GPU and returns the GPU with
    the most free memory that meets ``min_free_memory_mb``. The caller can use
    the returned device when loading its model and running inference. If CUDA is
    not available, this function returns a CPU device. The free-memory check is
    only a snapshot: it does not reserve memory or estimate a model's exact
    memory needs, so model loading can still fail later.

    @param caller[str] Name of the node requesting a device, for actionable errors.
    @param purpose[str] Description of the model or task that will use the device.
    @param min_free_memory_mb[int|None] Optional minimum free memory required for a GPU to be eligible. None allows any queryable GPU.
    @return torch.device The eligible GPU with the most free memory, or CPU if CUDA is unavailable.
    @throws ValueError If the requested memory threshold is negative.
    @throws CudaDeviceSelectionError If GPU memory cannot be queried or no GPU meets the threshold.
    """
    if min_free_memory_mb is not None and min_free_memory_mb < 0:
        raise ValueError("min_free_memory_mb must be non-negative")

    if not torch.cuda.is_available():
        return torch.device("cpu")

    device_count = torch.cuda.device_count()
    if device_count == 0:
        return torch.device("cpu")

    try:
        free_bytes = [
            torch.cuda.mem_get_info(index)[0] for index in range(device_count)
        ]
    except Exception as error:
        raise CudaDeviceSelectionError(
            f"{caller} could not choose a GPU for {purpose}: failed to query "
            f"CUDA memory ({error})."
        ) from error

    required_bytes = (min_free_memory_mb or 0) * 1024**2
    eligible = [
        index for index, free in enumerate(free_bytes) if free >= required_bytes
    ]
    if not eligible:
        requirement = (
            f"at least {min_free_memory_mb} MiB free"
            if min_free_memory_mb is not None
            else "a queryable CUDA device"
        )
        raise CudaDeviceSelectionError(
            f"{caller} cannot start {purpose}: no GPU meets the requirement "
            f"({requirement}). Available memory: "
            f"{_free_memory_report(device_count)}. Close other GPU workloads or "
            "lower this caller's configured threshold."
        )

    selected_index = max(eligible, key=free_bytes.__getitem__)
    return torch.device(f"cuda:{selected_index}")


def run_model_initializer(
    initializer: Callable[[], object],
    *,
    device: torch.device,
    caller: str,
    purpose: str,
) -> object:
    """! Load a model and add context if its CUDA allocation runs out.

    Calls ``initializer`` to perform the actual model loading or device move,
    then returns the loaded model. The caller supplies the device selected by
    ``select_cuda_device`` so this function can identify whether a loading
    failure occurred on CUDA. If PyTorch raises a CUDA out-of-memory
    ``RuntimeError``, this function raises ``CudaDeviceSelectionError`` with
    the node name, loading purpose, device, and a best-effort report of current
    GPU memory. Other exceptions are re-raised unchanged. This wrapper does not
    check or reserve memory before calling the initializer, and it does not
    retry loading on another GPU.

    @param initializer[Callable[[], object]] Function that loads and returns the model.
    @param device[torch.device] Device on which the initializer loads the model.
    @param caller[str] Name of the node initializing the model.
    @param purpose[str] Description of the model loading operation.
    @return object The model returned by the initializer.
    @throws CudaDeviceSelectionError If CUDA model loading runs out of memory.
    @throws RuntimeError Any other runtime error raised by the initializer.
    """
    try:
        return initializer()
    except RuntimeError as error:
        if device.type != "cuda" or "out of memory" not in str(error).lower():
            raise

        details = _free_memory_report(torch.cuda.device_count())
        raise CudaDeviceSelectionError(
            f"{caller} ran out of CUDA memory while {purpose} on {device}. "
            f"Memory available after the failed allocation: {details}. "
            "The pre-load free-memory check is only a snapshot; another process "
            "may allocate GPU memory before or during model loading."
        ) from error


if __name__ == "__main__":
    print(
        "Selected device:",
        select_cuda_device(caller="gpu.py demo", purpose="demonstration"),
    )

"""Package:   helpers
Filename:  gpu.py
Author:    Ron Shaked
Email:     ronshaked07@gmail.com
Copyright: 2021, Nova UTD
License:   MIT License

Select the GPU with the most free memory and add node context to failures.
Memory readings are snapshots; selection does not reserve memory for a model.
"""

from collections.abc import Callable
from typing import TypeVar

import torch

Model = TypeVar("Model")


def select_cuda_device(
    *,
    caller: str = "unspecified caller",
    purpose: str = "model initialization",
    min_free_memory_mb: int | None = None,
) -> torch.device:
    """! Return the visible GPU with the most free memory.

    Skip GPUs whose memory cannot be queried. Raise a descriptive error if
    CUDA cannot initialize, no GPUs are visible, or no readable GPU meets the
    optional minimum. This does not estimate model size or guarantee loading.

    @param caller[str] Node requesting a GPU, included in error messages.
    @param purpose[str] Model or operation that needs the GPU.
    @param min_free_memory_mb[int|None] Minimum free MiB; None means no minimum.
    @return torch.device Selected CUDA device.
    @throws ValueError If the minimum is negative.
    @throws RuntimeError If no suitable GPU can be selected.
    """
    if min_free_memory_mb is not None and min_free_memory_mb < 0:
        raise ValueError("min_free_memory_mb must be non-negative")

    context = f"{caller} cannot start {purpose}"
    try:
        # Initialize explicitly so driver failures retain their original message.
        torch.cuda.init()
        device_count = torch.cuda.device_count()
    except (RuntimeError, AssertionError) as error:
        raise RuntimeError(f"{context}: CUDA initialization failed: {error}") from error

    if device_count == 0:
        raise RuntimeError(
            f"{context}: no CUDA GPUs are visible. Check GPU access and "
            "CUDA_VISIBLE_DEVICES in this environment."
        )

    best_index = None
    best_free_bytes = -1
    report: list[str] = []
    for index in range(device_count):
        try:
            free_bytes, _ = torch.cuda.mem_get_info(index)
        except RuntimeError as error:
            report.append(f"cuda:{index}: memory query failed: {error}")
            continue
        report.append(f"cuda:{index}: {free_bytes / 1024**2:.0f} MiB free")
        if free_bytes > best_free_bytes:
            best_index, best_free_bytes = index, free_bytes

    if best_index is None:
        raise RuntimeError(
            f"{context}: all GPU memory queries failed. {'; '.join(report)}"
        )
    if best_free_bytes < (min_free_memory_mb or 0) * 1024**2:
        raise RuntimeError(
            f"{context}: no GPU with readable memory has the required "
            f"{min_free_memory_mb} MiB free. {'; '.join(report)}"
        )
    return torch.device(f"cuda:{best_index}")


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

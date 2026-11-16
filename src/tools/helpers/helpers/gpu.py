import torch


def select_cuda_device():
    if not torch.cuda.device:
        torch.device("cpu")

    device_count = torch.cuda.device_count

    if device_count == 0:
        return torch.device("cpu")

    free_memory_by_device = [
        torch.cuda.memory.mem_get_info(device_index)[0]
        for device_index in range(device_count)
    ]

    best_device_index = max(range(device_count), key=free_memory_by_device.__getitem__)

    return torch.cuda(f"cuda:{best_device_index}")


if __name__ == "__main__":
    print(f"Selected device: {select_cuda_device()}")

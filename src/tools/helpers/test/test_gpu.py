"""Fault-injection tests for GPU selection and descriptive loading errors."""

import unittest
from unittest.mock import patch

import torch

from helpers.gpu import (
    cuda_devices_by_free_memory,
    load_model_on_best_gpu,
    run_model_initializer,
    select_cuda_device,
)
from helpers.gpu import _free_bytes as _real_free_bytes


class GpuTest(unittest.TestCase):
    def setUp(self):
        # These tests must also run on machines without working CUDA hardware.
        for name, value in (("init", None), ("device_count", 3)):
            mock = patch(f"helpers.gpu.torch.cuda.{name}", return_value=value)
            mock.start()
            self.addCleanup(mock.stop)
        mock = patch("helpers.gpu._free_bytes")
        self.memory = mock.start()
        self.addCleanup(mock.stop)
        self.memory.side_effect = [2 * 1024**2, 5 * 1024**2, 1024**2]

    def test_selects_most_free_memory_at_threshold(self):
        self.assertEqual(
            select_cuda_device(min_free_memory_mb=5), torch.device("cuda:1")
        )

    def test_skips_failed_query(self):
        self.memory.side_effect = [RuntimeError("CUDA error 999"), 20, 10]
        self.assertEqual(select_cuda_device(), torch.device("cuda:1"))

    def test_all_queries_fail(self):
        self.memory.side_effect = RuntimeError("CUDA error 999")
        with self.assertRaisesRegex(
            RuntimeError, "lane node.*all GPU memory queries failed"
        ) as result:
            select_cuda_device(caller="lane node")
        for index in range(3):
            self.assertIn(
                f"cuda:{index}: memory query failed: CUDA error 999",
                str(result.exception),
            )

    def test_insufficient_memory(self):
        with self.assertRaisesRegex(
            RuntimeError, "required 6 MiB free.*cuda:1: 5 MiB free"
        ):
            select_cuda_device(min_free_memory_mb=6)

    def test_partial_queries_do_not_claim_all_gpus_are_full(self):
        self.memory.side_effect = [RuntimeError("CUDA error 999"), 0, 0]
        with self.assertRaisesRegex(
            RuntimeError, "no GPU with readable memory.*CUDA error 999"
        ):
            select_cuda_device(min_free_memory_mb=1)

    def test_no_devices(self):
        with patch("helpers.gpu.torch.cuda.device_count", return_value=0):
            with self.assertRaisesRegex(RuntimeError, "no CUDA GPUs are visible"):
                select_cuda_device()

    def test_initialization_failure_preserves_original_error(self):
        for error in (
            RuntimeError("CUDA error 999"),
            AssertionError("Torch not compiled with CUDA enabled"),
        ):
            with self.subTest(error=error), patch(
                "helpers.gpu.torch.cuda.init", side_effect=error
            ):
                with self.assertRaisesRegex(
                    RuntimeError, "lane node.*CUDA initialization failed"
                ) as result:
                    select_cuda_device(caller="lane node")
                self.assertIs(result.exception.__cause__, error)
                self.assertIn(str(error), str(result.exception))

    def test_negative_threshold(self):
        with self.assertRaises(ValueError):
            select_cuda_device(min_free_memory_mb=-1)

    def test_loader_returns_model(self):
        model = object()
        self.assertIs(
            run_model_initializer(
                lambda: model,
                device=torch.device("cuda:1"),
                caller="lane node",
                purpose="loading YOLO",
            ),
            model,
        )

    def test_loader_preserves_failures_with_context(self):
        for message in ("CUDA out of memory", "CUDA error 999", "model file not found"):
            error = RuntimeError(message)

            def load():
                raise error

            with self.subTest(message=message), self.assertRaises(
                RuntimeError
            ) as result:
                run_model_initializer(
                    load,
                    device=torch.device("cuda:1"),
                    caller="lane node",
                    purpose="loading YOLO",
                )
            self.assertEqual(
                str(result.exception),
                f"lane node failed while loading YOLO on cuda:1: {message}",
            )
            self.assertIs(result.exception.__cause__, error)

    def test_orders_candidates_by_free_memory(self):
        self.assertEqual(
            cuda_devices_by_free_memory(min_free_memory_mb=2),
            [torch.device("cuda:1"), torch.device("cuda:0")],
        )

    def test_selection_never_creates_cuda_contexts(self):
        # mem_get_info creates a CUDA context (~150 MiB) on every GPU probed and
        # fails on a full GPU, so selection must read memory through NVML.
        self.memory.side_effect = None
        with patch("helpers.gpu._free_bytes", wraps=_real_free_bytes), patch(
            "helpers.gpu.pynvml"
        ) as nvml, patch("helpers.gpu.torch.cuda.get_device_properties") as properties, patch(
            "helpers.gpu.torch.cuda.mem_get_info",
            side_effect=AssertionError("context created"),
        ):
            properties.side_effect = lambda index: type("P", (), {"uuid": f"u{index}"})
            nvml.NVMLError = Exception
            nvml.nvmlDeviceGetHandleByUUID.side_effect = lambda uuid: uuid
            nvml.nvmlDeviceGetMemoryInfo.side_effect = lambda handle: type(
                "M", (), {"free": {b"GPU-u0": 1, b"GPU-u1": 9, b"GPU-u2": 4}[handle]}
            )
            self.assertEqual(select_cuda_device(), torch.device("cuda:1"))

    def test_load_falls_back_when_gpu_context_cannot_be_created(self):
        tried = []

        def load(device):
            tried.append(str(device))
            if device == torch.device("cuda:1"):
                raise RuntimeError(
                    "CUDA error: CUDA-capable device(s) is/are busy or unavailable\n"
                    "Returning 999 (CUDA_ERROR_UNKNOWN) from cuDevicePrimaryCtxRetain"
                )
            return "model"

        with patch("helpers.gpu.torch.cuda.empty_cache"):
            model, device = load_model_on_best_gpu(
                load, caller="seg node", purpose="loading PSPNet", log=lambda _: None
            )
        self.assertEqual((model, device), ("model", torch.device("cuda:0")))
        self.assertEqual(tried, ["cuda:1", "cuda:0"])

    def test_load_falls_back_to_next_gpu_on_out_of_memory(self):
        tried = []

        def load(device):
            tried.append(str(device))
            if device == torch.device("cuda:1"):
                raise RuntimeError("CUDA error: out of memory")
            return "model"

        with patch("helpers.gpu.torch.cuda.empty_cache"):
            model, device = load_model_on_best_gpu(
                load, caller="lane node", purpose="loading YOLO", log=lambda _: None
            )
        self.assertEqual((model, device), ("model", torch.device("cuda:0")))
        self.assertEqual(tried, ["cuda:1", "cuda:0"])

    def test_load_raises_when_every_gpu_is_out_of_memory(self):
        def load(device):
            raise torch.cuda.OutOfMemoryError("CUDA out of memory")

        with patch("helpers.gpu.torch.cuda.empty_cache"), self.assertRaisesRegex(
            RuntimeError, "lane node.*every candidate GPU was full or unavailable"
        ):
            load_model_on_best_gpu(
                load, caller="lane node", purpose="loading YOLO", log=lambda _: None
            )

    def test_load_does_not_retry_other_failures(self):
        tried = []

        def load(device):
            tried.append(device)
            raise RuntimeError("model file not found")

        with self.assertRaisesRegex(RuntimeError, "model file not found"):
            load_model_on_best_gpu(
                load, caller="lane node", purpose="loading YOLO", log=lambda _: None
            )
        self.assertEqual(len(tried), 1)


if __name__ == "__main__":
    unittest.main()

"""Fault-injection tests for GPU selection and descriptive loading errors."""

import unittest
from unittest.mock import patch

import torch

from helpers.gpu import run_model_initializer, select_cuda_device


class GpuTest(unittest.TestCase):
    def setUp(self):
        # These tests must also run on machines without working CUDA hardware.
        for name, value in (("init", None), ("device_count", 3)):
            mock = patch(f"helpers.gpu.torch.cuda.{name}", return_value=value)
            mock.start()
            self.addCleanup(mock.stop)
        mock = patch("helpers.gpu.torch.cuda.mem_get_info")
        self.memory = mock.start()
        self.addCleanup(mock.stop)
        self.memory.side_effect = [(2 * 1024**2, 0), (5 * 1024**2, 0), (1024**2, 0)]

    def test_selects_most_free_memory_at_threshold(self):
        self.assertEqual(
            select_cuda_device(min_free_memory_mb=5), torch.device("cuda:1")
        )

    def test_skips_failed_query(self):
        self.memory.side_effect = [RuntimeError("CUDA error 999"), (20, 30), (10, 30)]
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
        self.memory.side_effect = [RuntimeError("CUDA error 999"), (0, 30), (0, 30)]
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


if __name__ == "__main__":
    unittest.main()

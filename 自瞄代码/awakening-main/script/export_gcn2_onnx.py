#!/usr/bin/env python3
import argparse
import inspect
import json
import zipfile
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn


class GCNv2Dense(nn.Module):
    def __init__(self):
        super().__init__()
        self.elu = nn.ELU(inplace=True)
        self.conv1 = nn.Conv2d(1, 32, kernel_size=4, stride=2, padding=1)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=1)
        self.conv3_1 = nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=1)
        self.conv3_2 = nn.Conv2d(128, 128, kernel_size=4, stride=2, padding=1)
        self.conv4_1 = nn.Conv2d(128, 256, kernel_size=3, stride=1, padding=1)
        self.conv4_2 = nn.Conv2d(256, 256, kernel_size=4, stride=2, padding=1)
        self.convF_1 = nn.Conv2d(256, 256, kernel_size=3, stride=1, padding=1)
        self.convF_2 = nn.Conv2d(256, 256, kernel_size=1, stride=1, padding=0)
        self.convD_1 = nn.Conv2d(256, 256, kernel_size=3, stride=1, padding=1)
        self.convD_2 = nn.Conv2d(256, 256, kernel_size=1, stride=1, padding=0)
        self.pixel_shuffle = nn.PixelShuffle(16)

    def forward(self, x):
        x = self.elu(self.conv1(x))
        x = self.elu(self.conv2(x))
        x = self.elu(self.conv3_1(x))
        x = self.elu(self.conv3_2(x))
        x = self.elu(self.conv4_1(x))
        x = self.elu(self.conv4_2(x))

        desc = self.elu(self.convF_1(x))
        desc = self.convF_2(desc)
        desc_norm = torch.norm(desc, p=2, dim=1, keepdim=True)
        desc = desc / desc_norm

        det = self.elu(self.convD_1(x))
        det = torch.sigmoid(self.convD_2(det))
        return desc, det


def _archive_root(names):
    for name in names:
        if name.endswith("/model.json"):
            return name[: -len("model.json")]
    raise RuntimeError("model.json not found in legacy TorchScript archive")


def _tensor_from_archive(zip_file, root, tensor_meta):
    dims = tuple(int(v) for v in tensor_meta["dims"])
    dtype = tensor_meta["dataType"]
    key = tensor_meta["data"]["key"]
    raw = zip_file.read(root + key)
    if dtype == "FLOAT":
        array = np.frombuffer(raw, dtype="<f4").copy()
    else:
        raise RuntimeError(f"unsupported tensor dtype: {dtype}")
    return torch.from_numpy(array.reshape(dims))


def load_legacy_gcnv2(pt_path):
    model = GCNv2Dense()
    tensor_to_param = {
        3: "conv1.weight",
        4: "conv1.bias",
        5: "conv2.weight",
        6: "conv2.bias",
        7: "conv3_1.weight",
        8: "conv3_1.bias",
        9: "conv3_2.weight",
        10: "conv3_2.bias",
        11: "conv4_1.weight",
        12: "conv4_1.bias",
        13: "conv4_2.weight",
        14: "conv4_2.bias",
        15: "convF_1.weight",
        16: "convF_1.bias",
        17: "convF_2.weight",
        18: "convF_2.bias",
        19: "convD_1.weight",
        20: "convD_1.bias",
        21: "convD_2.weight",
        22: "convD_2.bias",
    }

    with zipfile.ZipFile(pt_path) as zip_file:
        root = _archive_root(zip_file.namelist())
        metadata = json.loads(zip_file.read(root + "model.json"))
        state_dict = model.state_dict()
        for tensor_index, param_name in tensor_to_param.items():
            state_dict[param_name] = _tensor_from_archive(
                zip_file,
                root,
                metadata["tensors"][tensor_index],
            )

    model.load_state_dict(state_dict)
    model.eval()
    return model


def main():
    parser = argparse.ArgumentParser(
        description="Export legacy GCNv2_SLAM TorchScript archive to dense ONNX."
    )
    parser.add_argument(
        "--pt",
        default="/home/hy/paper_work/GCNv2_SLAM/GCN2/gcn2_640x480.pt",
        help="legacy .pt archive path",
    )
    parser.add_argument(
        "--onnx",
        default="/home/hy/paper_work/GCNv2_SLAM/GCN2/gcn2_640x480.onnx",
        help="output ONNX path",
    )
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--opset", type=int, default=11)
    args = parser.parse_args()

    onnx_path = Path(args.onnx)
    onnx_path.parent.mkdir(parents=True, exist_ok=True)

    model = load_legacy_gcnv2(args.pt)
    dummy = torch.zeros(1, 1, args.height, args.width, dtype=torch.float32)
    with torch.no_grad():
        export_kwargs = {
            "input_names": ["image"],
            "output_names": ["descriptors_dense", "detector_cells"],
            "opset_version": args.opset,
            "do_constant_folding": True,
        }
        if "dynamo" in inspect.signature(torch.onnx.export).parameters:
            export_kwargs["dynamo"] = False
        torch.onnx.export(model, dummy, str(onnx_path), **export_kwargs)

    print(f"exported dense ONNX: {onnx_path}")


if __name__ == "__main__":
    main()

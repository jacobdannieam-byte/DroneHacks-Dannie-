"""Train the optional A2 basis with differentiable ridge on causal D2 windows."""

import argparse
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path

import numpy as np
import torch
from torch import nn

from examples.a1_data import digest, fingerprints
from examples.d2_data import episodes, training_arrays
from refnav.dynamics.adaptive_features import FeatureBasis, FORGET, LOWER, MIN_ROWS, PRIOR, UPPER, VERSION
from refnav.dynamics.history import ACCELERATION_SCALE, VELOCITY_SCALE
from refnav.dynamics.residual import RESIDUAL_LIMITS

SEED = 202611370
EPOCHS = 12
BATCH_SIZE = 384
LEARNING_RATE = .003
CHECKPOINT = "feature_basis.npz"


class TrainBasis(nn.Module):
    def __init__(self):
        super().__init__()
        self.hidden = nn.Linear(8, 16)
        self.out = nn.Linear(16, 8)

    def design(self, nominal, rate, *, linear_only=False):
        hidden = torch.relu(self.hidden(torch.cat((nominal, rate), dim=-1)))
        learned = torch.tanh(self.out(hidden)).reshape(*nominal.shape, 2)
        if linear_only:
            learned = torch.zeros_like(learned)
        activity = torch.clamp(nominal.abs() + rate.abs(), max=1.)
        return torch.cat((nominal[..., None], -rate[..., None], learned * activity[..., None]), dim=-1)

    def forward(self, nominal, rate, windows, activity, *, linear_only=False):
        historic_nominal = windows[..., :4]
        historic_rate = windows[..., 8:12]
        design = self.design(historic_nominal, historic_rate, linear_only=linear_only)
        residual = windows[..., 4:8].clamp(-1., 1.) - historic_nominal
        weights = FORGET ** torch.arange(windows.shape[1] - 1, -1, -1, dtype=windows.dtype)
        weights = weights * windows[..., -1]
        gram = torch.einsum("bn,bnai,bnaj->baij", weights, design, design)
        rhs = torch.einsum("bn,bnai,bna->bai", weights, design, residual)
        ridge = torch.diag(windows.new_tensor(PRIOR))
        coeff = torch.linalg.solve(gram + ridge, rhs[..., None])[..., 0]
        coeff = torch.clamp(coeff, min=windows.new_tensor(LOWER), max=windows.new_tensor(UPPER))
        features = self.design(nominal, rate, linear_only=linear_only)
        delta = (features * coeff).sum(dim=-1).clamp(-.5, .5) * activity
        return (nominal + delta).clamp(-1., 1.) - nominal

    def export(self):
        return {"w1": self.hidden.weight.detach().numpy().T.copy(), "b1": self.hidden.bias.detach().numpy().copy(),
                "w2": self.out.weight.detach().numpy().T.copy(), "b2": self.out.bias.detach().numpy().copy()}


def arrays(cases):
    """Reuse D2's train/validation labels and its pre-observation window order."""
    inputs, windows, activity, targets = training_arrays(cases)
    usable = windows[:, :, -1].sum(axis=1) >= MIN_ROWS
    inputs, windows, activity, targets = (item[usable] for item in (inputs, windows, activity, targets))
    nominal = inputs[:, (9, 10, 11, 15)] / np.asarray(ACCELERATION_SCALE, np.float32)
    rate = np.clip(inputs[:, (0, 1, 2, 13)] / np.asarray(VELOCITY_SCALE, np.float32), -3, 3)
    normalized = targets * (np.asarray(RESIDUAL_LIMITS, np.float32) / ACCELERATION_SCALE).astype(np.float32)
    return tuple(torch.from_numpy(item) for item in (nominal, rate, windows, activity, normalized))


def validation_loss(network, data, *, linear_only=False):
    count = len(data[-1])
    total = 0.
    with torch.inference_mode():
        for start in range(0, count, BATCH_SIZE):
            *inputs, targets = (item[start:start + BATCH_SIZE] for item in data)
            total += (network(*inputs, linear_only=linear_only) - targets).square().sum().item()
    return total / data[-1].numel()


def fit(train, validation):
    torch.manual_seed(SEED)
    network = TrainBasis()
    optimizer = torch.optim.Adam(network.parameters(), lr=LEARNING_RATE)
    history = [validation_loss(network, validation)]
    epoch = 0
    best = deepcopy(network.state_dict())
    linear_error = validation_loss(network, validation, linear_only=True)
    for index in range(1, EPOCHS + 1):
        for batch in torch.randperm(len(train[-1])).split(BATCH_SIZE):
            *inputs, targets = (item[batch] for item in train)
            optimizer.zero_grad(set_to_none=True)
            loss = (network(*inputs) - targets).square().mean()
            loss.backward()
            optimizer.step()
        history.append(validation_loss(network, validation))
        if history[index] < history[epoch]:
            epoch, best = index, deepcopy(network.state_dict())
    network.load_state_dict(best)
    return network, epoch, history, linear_error


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifacts", type=Path, default=Path("artifacts"))
    parser.add_argument("--output", type=Path, required=True, help="new checkpoint output directory")
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("output directory already exists")
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    torch.use_deterministic_algorithms(True)
    d2_path = args.artifacts / "adaptive_dynamics_d2" / "training.json"
    d2 = json.loads(d2_path.read_text(encoding="utf-8"))
    cases = {name: episodes(name) for name in ("train", "validation")}
    fingerprints_by_split = {name: [fingerprints(episode) for episode in items] for name, items in cases.items()}
    for name, items in fingerprints_by_split.items():
        if [entry["response"] for entry in items] != d2["response_ids"][name]:
            raise ValueError("A2 training split differs from the verified D2 manifest")
    for key in ("trace", "response", "regimes"):
        train_items, val_items = ([entry[key] for entry in items] for items in fingerprints_by_split.values())
        if key == "regimes":
            train_items = [item for group in train_items for item in group]
            val_items = [item for group in val_items for item in group]
        if set(train_items) & set(val_items):
            raise ValueError(f"train and validation overlap by {key}")
    train, validation = (arrays(cases[name]) for name in ("train", "validation"))
    network, epoch, history, linear_error = fit(train, validation)
    args.output.mkdir(parents=True)
    checkpoint = args.output / CHECKPOINT
    np.savez(checkpoint, version=np.int32(VERSION), **network.export())
    exported = FeatureBasis.load(checkpoint)
    nominal, rate = (item[:128] for item in validation[:2])
    with torch.inference_mode():
        expected = network.design(nominal, rate).numpy()
    actual = exported.design(nominal.numpy(), rate.numpy())
    np.testing.assert_allclose(actual, expected, atol=2e-6, rtol=1e-6)
    record = {"seed": SEED, "epochs": EPOCHS, "batch_size": BATCH_SIZE, "learning_rate": LEARNING_RATE,
              "torch": torch.__version__, "numpy": np.__version__, "device": "cpu", "threads": 1,
              "target": "normalized one-step capped acceleration residual, using causal D2 windows",
              "selected_epoch": epoch, "validation_mse": history[epoch],
              "validation_mse_by_epoch": history, "linear_only_validation_mse": linear_error,
              "train_samples": len(train[-1]), "validation_samples": len(validation[-1]),
              "split_fingerprints": fingerprints_by_split,
              "d2_training_manifest_sha256": digest(d2),
              "checkpoint_sha256": sha256(checkpoint.read_bytes()).hexdigest(),
              "checkpoint_bytes": checkpoint.stat().st_size,
              "parameter_count": sum(item.numel() for item in network.parameters()),
              "max_export_error": float(np.max(np.abs(expected - actual)))}
    (args.output / "training.json").write_text(json.dumps(record, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"A2 epoch {epoch}, validation MSE {history[epoch]:.8f}, linear ablation {linear_error:.8f}", flush=True)


if __name__ == "__main__":
    main()

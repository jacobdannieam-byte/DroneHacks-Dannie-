"""Train candidate D2 encoders on CPU and export portable frozen NumPy checkpoints."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch
from torch import nn

from examples.d2_data import SEED, episodes, training_arrays
from refnav.dynamics.adaptive import AdaptiveNetwork, CONTEXT_SIZE, ENCODERS, HEAD_WIDTHS, MODEL_VERSION
from refnav.dynamics.history import HISTORY_FEATURES, HISTORY_STEPS
from refnav.dynamics.residual import FEATURE_LIMITS, RESIDUAL_LIMITS

EPOCHS = 18
BATCH = 384


class TrainNetwork(nn.Module):
    def __init__(self, encoder):
        super().__init__()
        self.encoder = encoder
        width = 32 if encoder == "mlp" else int(encoder[3:])
        if encoder == "mlp":
            self.enc = nn.Linear(HISTORY_STEPS * HISTORY_FEATURES, width)
        else:
            self.gru = nn.GRU(HISTORY_FEATURES, width, batch_first=True)
        self.latent = nn.Linear(width, CONTEXT_SIZE)
        self.head1 = nn.Linear(len(FEATURE_LIMITS) + CONTEXT_SIZE, HEAD_WIDTHS[0])
        self.head2 = nn.Linear(*HEAD_WIDTHS)
        self.out = nn.Linear(HEAD_WIDTHS[1], 4)
        nn.init.zeros_(self.out.weight)
        nn.init.zeros_(self.out.bias)

    def encode(self, windows):
        if self.encoder == "mlp":
            encoded = torch.relu(self.enc(windows.flatten(1)))
        else:
            _, hidden = self.gru(windows)
            encoded = hidden[0]
        return torch.tanh(self.latent(encoded))

    def forward(self, inputs, windows):
        features = torch.cat((inputs, self.encode(windows)), dim=1)
        first = torch.relu(self.head1(features))
        second = torch.relu(self.head2(first))
        return torch.tanh(self.out(second))

    def export(self, mean, scale):
        result = {"mean": mean, "scale": scale}
        for name in ("latent", "head1", "head2", "out") + (("enc",) if self.encoder == "mlp" else ()):
            layer = getattr(self, name)
            result[f"{name}_w"] = layer.weight.detach().numpy().copy()
            result[f"{name}_b"] = layer.bias.detach().numpy().copy()
        if self.encoder != "mlp":
            gru_names = {"gru_ih": "weight_ih_l0", "gru_hh": "weight_hh_l0",
                         "gru_bi": "bias_ih_l0", "gru_bh": "bias_hh_l0"}
            for name, attribute in gru_names.items():
                result[name] = getattr(self.gru, attribute).detach().numpy().copy()
        return result


def fit(encoder, train, validation):
    torch.manual_seed(SEED)
    network = TrainNetwork(encoder)
    optimizer = torch.optim.Adam(network.parameters(), lr=0.002)
    n = len(train[0])

    def validation_loss():
        total = 0.0
        with torch.inference_mode():
            for start in range(0, len(validation[0]), BATCH):
                x, h, g, y = (value[start:start + BATCH] for value in validation)
                total += (network(x, h) * g - y).square().sum().item()
        return total / validation[-1].numel()

    best_loss = validation_loss()
    losses = [best_loss]
    best = {name: value.detach().clone() for name, value in network.state_dict().items()}
    selected_epoch = 0
    for epoch in range(1, EPOCHS + 1):
        for batch in torch.randperm(n).split(BATCH):
            x, h, g, y = (value[batch] for value in train)
            optimizer.zero_grad(set_to_none=True)
            loss = (network(x, h) * g - y).square().mean()
            loss.backward()
            optimizer.step()
        loss = validation_loss()
        losses.append(loss)
        if loss < best_loss:
            best_loss, selected_epoch = loss, epoch
            best = {name: value.detach().clone() for name, value in network.state_dict().items()}
    network.load_state_dict(best)
    return network, selected_epoch, losses


def verify_export(network, portable, samples, mean, scale):
    inputs, windows = samples[:2]
    indices = np.linspace(0, len(inputs) - 1, 32, dtype=int)
    with torch.inference_mode():
        x = torch.from_numpy((inputs[indices] - mean) / scale)
        h = torch.from_numpy(windows[indices])
        expected = network(x, h).numpy() * RESIDUAL_LIMITS
    measured = np.array([portable.correction(inputs[i], portable.encode(windows[i])) for i in indices])
    np.testing.assert_allclose(measured, expected, atol=3e-6, rtol=1e-5)
    return float(np.abs(measured - expected).max())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("artifacts/adaptive_dynamics_d2"))
    args = parser.parse_args()
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    torch.use_deterministic_algorithms(True)
    training, validation = episodes("train"), episodes("validation")
    fingerprints = {name: [ep.response_id() for ep in cases] for name, cases in
                    (("train", training), ("validation", validation))}
    if len(set(fingerprints["train"] + fingerprints["validation"])) != len(training) + len(validation):
        raise ValueError("response configurations overlap")
    train_raw, valid_raw = training_arrays(training), training_arrays(validation)
    mean = train_raw[0].mean(axis=0)
    scale = np.maximum(train_raw[0].std(axis=0), 0.001)

    def tensors(raw):
        return tuple(torch.from_numpy(value) for value in ((raw[0] - mean) / scale, *raw[1:]))

    train, valid = tensors(train_raw), tensors(valid_raw)
    args.output.mkdir(parents=True, exist_ok=True)
    candidates = {}
    for encoder in ENCODERS:
        network, epoch, losses = fit(encoder, train, valid)
        path = args.output / f"adaptive_{encoder}.npz"
        np.savez(path, version=np.int32(MODEL_VERSION), encoder=np.array(encoder), **network.export(mean, scale))
        portable = AdaptiveNetwork.load(path)
        parity = verify_export(network, portable, valid_raw, mean, scale)
        candidates[encoder] = {"checkpoint": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                               "parameters": sum(parameter.numel() for parameter in network.parameters()),
                               "bytes": path.stat().st_size, "selected_epoch": epoch,
                               "validation_normalized_mse": losses[epoch], "validation_history": losses,
                               "max_torch_numpy_difference": parity}
        print(f"{encoder}: selected epoch {epoch}, validation MSE {losses[epoch]:.6f}", flush=True)
    selected = min(candidates, key=lambda name: candidates[name]["validation_normalized_mse"])
    record = {"seed": SEED, "torch": torch.__version__, "numpy": np.__version__, "cpu_threads": 1,
              "epochs": EPOCHS, "batch_size": BATCH, "learning_rate": 0.002,
              "response_ids": fingerprints, "training_transitions": len(train[0]),
              "validation_transitions": len(valid[0]), "selected_encoder": selected, "candidates": candidates,
              "selection": "Lowest held-out validation normalized residual MSE, including epoch zero"}
    (args.output / "training.json").write_text(json.dumps(record, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Selected {selected}. Wrote checkpoints and training.json to {args.output}", flush=True)


if __name__ == "__main__":
    main()

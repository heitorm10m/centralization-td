from __future__ import annotations

from .models import WellModel


def trajectory_plan_view(well: WellModel) -> list[tuple[float, float]]:
    return [(node.easting_m, node.northing_m) for node in well.derived_nodes()]


def plot_plan_view(well: WellModel) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("matplotlib is optional; install it before using plotting helpers.") from exc

    coordinates = trajectory_plan_view(well)
    xs = [point[0] for point in coordinates]
    ys = [point[1] for point in coordinates]

    plt.figure()
    plt.plot(xs, ys, marker="o")
    plt.xlabel("Easting [m]")
    plt.ylabel("Northing [m]")
    plt.title(well.name)
    plt.axis("equal")
    plt.tight_layout()
    plt.show()

from __future__ import annotations

from collections import defaultdict, deque
from pathlib import Path
from typing import Iterable

from plan import AbstractNode, ActionNode, DAG, DelayNode, StartNode, WaitNode
from plan.dag import children


class DAGPlotError(RuntimeError):
    """DAG 绘制失败。"""


def draw_dag(
    dag: DAG | Iterable[AbstractNode],
    output: str | Path,
    *,
    title: str = "Plan DAG",
    dpi: int = 160,
) -> Path:
    """用 matplotlib 绘制 DAG，返回写出的图片路径。"""

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import FancyBboxPatch
    except ImportError as exc:
        raise DAGPlotError("绘制 DAG 需要安装 matplotlib。") from exc

    nodes = list(dag.nodes if isinstance(dag, DAG) else dag)
    if not nodes:
        raise DAGPlotError("DAG 至少需要一个节点。")

    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    layers = _layers(nodes)
    positions = _positions(layers)
    width = max(8.0, 2.8 * len(layers))
    height = max(4.5, 1.2 * max(len(layer) for layer in layers.values()))
    fig, ax = plt.subplots(figsize=(width, height), dpi=dpi)
    ax.set_title(title, fontsize=14, pad=16)
    ax.axis("off")

    node_size = (1.9, 0.62)
    child_map = children(nodes)
    for dep in nodes:
        x0, y0 = positions[dep]
        for node in child_map[dep]:
            x1, y1 = positions[node]
            ax.annotate(
                "",
                xy=(x1 - node_size[0] / 2, y1),
                xytext=(x0 + node_size[0] / 2, y0),
                arrowprops={
                    "arrowstyle": "->",
                    "color": "#59636e",
                    "linewidth": 1.3,
                    "shrinkA": 4,
                    "shrinkB": 4,
                },
                zorder=1,
            )

    for node in nodes:
        x, y = positions[node]
        face, edge = _node_colors(node)
        rect = FancyBboxPatch(
            (x - node_size[0] / 2, y - node_size[1] / 2),
            node_size[0],
            node_size[1],
            boxstyle="round,pad=0.04,rounding_size=0.08",
            linewidth=1.3,
            edgecolor=edge,
            facecolor=face,
            zorder=2,
        )
        ax.add_patch(rect)
        ax.text(
            x,
            y,
            _label(node),
            ha="center",
            va="center",
            fontsize=9,
            color="#101820",
            linespacing=1.18,
            zorder=3,
        )

    xs = [pos[0] for pos in positions.values()]
    ys = [pos[1] for pos in positions.values()]
    ax.set_xlim(min(xs) - 1.4, max(xs) + 1.4)
    ax.set_ylim(min(ys) - 0.9, max(ys) + 0.9)
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    return output_path


def _layers(nodes: list[AbstractNode]) -> dict[int, list[AbstractNode]]:
    node_set = set(nodes)
    for node in nodes:
        for dep in node.deps:
            if dep not in node_set:
                raise DAGPlotError(f"节点 {node.name} 依赖了 DAG 外节点 {dep.name}。")

    indegree = {node: len(node.deps) for node in nodes}
    depth = {node: 0 for node in nodes}
    child_map = children(nodes)
    queue = deque(node for node in nodes if indegree[node] == 0)
    visited = 0

    while queue:
        node = queue.popleft()
        visited += 1
        for child in child_map[node]:
            depth[child] = max(depth[child], depth[node] + 1)
            indegree[child] -= 1
            if indegree[child] == 0:
                queue.append(child)

    if visited != len(nodes):
        raise DAGPlotError("DAG 中存在环，无法绘制。")

    layers: dict[int, list[AbstractNode]] = defaultdict(list)
    for node in nodes:
        layers[depth[node]].append(node)
    return dict(sorted(layers.items()))


def _positions(layers: dict[int, list[AbstractNode]]) -> dict[AbstractNode, tuple[float, float]]:
    positions: dict[AbstractNode, tuple[float, float]] = {}
    for layer_index, layer_nodes in layers.items():
        y0 = (len(layer_nodes) - 1) / 2
        for row, node in enumerate(layer_nodes):
            positions[node] = (layer_index * 2.8, y0 - row)
    return positions


def _node_colors(node: AbstractNode) -> tuple[str, str]:
    if isinstance(node, ActionNode):
        if node.kind == "chassis":
            return "#dcecff", "#3974b8"
        if node.kind == "arm":
            return "#dff4e8", "#2d8f5b"
        if node.kind == "flags":
            return "#fff1d6", "#c77a13"
        raise DAGPlotError(f"未知 ActionNode kind：{node.kind}")
    if isinstance(node, WaitNode):
        return "#eceff3", "#6b7785"
    if isinstance(node, DelayNode):
        return "#f3e8ff", "#8558c8"
    if isinstance(node, StartNode):
        return "#ffffff", "#2f3a45"
    if type(node) is AbstractNode:
        return "#ffffff", "#2f3a45"
    raise DAGPlotError(f"未知节点类型：{node!r}")


def _label(node: AbstractNode) -> str:
    if isinstance(node, ActionNode):
        duration = float(node.path.duration) if node.path is not None else 0.0
        return f"{node.name}\n{node.kind}  {duration:.2f}s"
    if isinstance(node, WaitNode):
        timeout = "inf" if node.timeout == 0 else f"{float(node.timeout):.2f}s"
        return f"{node.name}\nwait  {timeout}"
    if isinstance(node, DelayNode):
        return f"{node.name}\ndelay  {float(node.duration):.2f}s"
    if isinstance(node, StartNode):
        return f"{node.name}\nstart"
    return node.name

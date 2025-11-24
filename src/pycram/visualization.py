from __future__ import annotations

from typing import Any, Callable, Dict, Iterable, Optional, Sequence, Tuple


def plot_rustworkx_interactive(
    graph: "Any",
    *,
    node_params: Optional[Dict[int, Dict[str, Any]]] = None,
    node_label: Optional[Callable[[int, Any], str]] = None,
    attributes: Optional[Sequence[str]] = None,
    layout: str = "spring",
    start: Optional[int] = None,
    title: str = "Rustworkx Graph",
    width: int = 1200,
    height: int = 800,
):
    """
    Plot an interactive visualization of a rustworkx graph.

    - Click on a node to show its parameters in a side panel.
    - Hover shows the node label.

    Parameters
    ----------
    graph:
        A rustworkx.PyGraph or rustworkx.PyDiGraph instance.
    node_params:
        Optional mapping from node index to a dict of parameters to display when
        the node is clicked. If not provided and the node payload is a dict,
        those items will be used. If provided together with ``attributes``, the
        displayed parameters will be filtered to the given attribute names.
    node_label:
        Optional callable that takes (index, payload) and returns a label string
        for the node. By default it tries to use ``payload.get('label')`` or
        ``str(payload)``.
    attributes:
        Optional list of attribute names to show from the parameters. Ignored if
        parameters are not dict-like.
    layout:
        Layout algorithm to use: "spring", "kamada_kawai", or "bfs".
    start:
        Optional start node index for "bfs" layout.
    title:
        Plot title.
    width, height:
        Figure size in pixels.

    Notes
    -----
    This function imports bokeh lazily so that it does not add a hard runtime
    dependency unless you call it. Install with `pip install bokeh`.
    """

    # Local imports to keep dependency optional at import time.
    try:
        import networkx as nx
        import importlib
        bokeh_layouts = importlib.import_module("bokeh.layouts")
        bokeh_models = importlib.import_module("bokeh.models")
        bokeh_plotting = importlib.import_module("bokeh.plotting")
        row = getattr(bokeh_layouts, "row")
        ColumnDataSource = getattr(bokeh_models, "ColumnDataSource")
        Div = getattr(bokeh_models, "Div")
        HoverTool = getattr(bokeh_models, "HoverTool")
        NodesAndLinkedEdges = getattr(bokeh_models, "NodesAndLinkedEdges")
        TapTool = getattr(bokeh_models, "TapTool")
        CustomJS = getattr(bokeh_models, "CustomJS")
        figure = getattr(bokeh_plotting, "figure")
        from_networkx = getattr(bokeh_plotting, "from_networkx")
        show = getattr(bokeh_plotting, "show")
    except Exception as exc:  # pragma: no cover - informative error only if used
        raise RuntimeError(
            "plot_rustworkx_interactive requires bokeh and networkx. Install with 'pip install bokeh networkx'."
        ) from exc

    # Import typed only to avoid hard dependency in module scope
    try:
        import rustworkx as rx  # type: ignore
    except Exception:
        rx = None  # best effort; we don't need types here

    # Build a NetworkX graph from rustworkx graph
    is_directed = getattr(graph, "is_directed", lambda: True)()
    nx_g = nx.DiGraph() if is_directed else nx.Graph()

    # rustworkx nodes are indexed 0..n-1. Access via graph.nodes(), graph.node_indices() or graph.num_nodes()
    # We'll iterate over range(num_nodes) and get payload via graph[node]
    num_nodes = getattr(graph, "num_nodes")()

    def get_node_payload(idx: int) -> Any:
        # PyGraph/PyDiGraph index access returns payload
        try:
            return graph[idx]
        except Exception:
            return None

    # Prepare label/params for each node
    attributes = list(attributes) if attributes is not None else None

    for i in range(num_nodes):
        payload = get_node_payload(i)
        # Label
        if node_label is not None:
            label = node_label(i, payload)
        else:
            label = None
            if isinstance(payload, dict) and "label" in payload:
                label = str(payload.get("label"))
            if label is None:
                label = str(payload)
        # Parameters
        params = None
        if node_params is not None:
            params = node_params.get(i)
        else:
            params = _object_params_with_properties(payload)
        # Filter attributes if requested
        if attributes is not None and isinstance(params, dict):
            params = {k: params.get(k) for k in attributes if k in params}
        # Attach as node attributes
        nx_g.add_node(
            i,
            label=label,
            param_text=_format_params(params),
        )

    # Add edges
    for (u, v) in getattr(graph, "edge_list")():
        nx_g.add_edge(u, v)

    # Choose layout
    if layout == "spring":
        pos = getattr(nx, "spring_layout")(nx_g, seed=42)
    elif layout == "kamada_kawai":
        pos = getattr(nx, "kamada_kawai_layout")(nx_g)
    elif layout == "bfs":
        if start is None and num_nodes > 0:
            start = 0
        pos = getattr(nx, "bfs_layout")(nx_g, start=start)
    else:
        pos = getattr(nx, "spring_layout")(nx_g, seed=42)

    # Create bokeh figure
    p = figure(
        title=title,
        x_axis_location=None,
        y_axis_location=None,
        width=width,
        height=height,
        toolbar_location="below",
        background_fill_color="#efefef",
    )
    p.grid.grid_line_color = None

    # Build graph renderer via from_networkx
    g_renderer = from_networkx(nx_g, pos)

    # Node glyphs and hover
    g_renderer.node_renderer.glyph.update(size=18, fill_color="#79a6d2")
    hover = HoverTool(tooltips=[("label", "@label")])
    p.add_tools(hover, TapTool())
    g_renderer.selection_policy = NodesAndLinkedEdges()
    g_renderer.inspection_policy = NodesAndLinkedEdges()

    # Prepare a side panel for parameters
    info = Div(text="<b>Click a node to see its parameters</b>", width=400, height=height)

    # Ensure param_text exists on data source
    # from_networkx created a CDS with 'index' only; merge our attrs
    cds = g_renderer.node_renderer.data_source
    # Extract param_texts and labels in the same order as 'index'
    index_list = list(cds.data.get("index", []))
    labels = [nx_g.nodes[i].get("label", str(i)) for i in index_list]
    params_html = [nx_g.nodes[i].get("param_text", "") for i in index_list]

    # Update CDS with fields used by JS callback and hover
    cds.data["label"] = labels
    cds.data["param_text"] = params_html

    # JS callback to update the Div when selection changes
    callback = CustomJS(
        args=dict(source=cds, panel=info),
        code="""
            const inds = source.selected.indices;
            if (inds.length === 0) {
                panel.text = "<b>Click a node to see its parameters</b>";
                return;
            }
            const i = inds[0];
            const label = source.data['label'][i];
            const params = source.data['param_text'][i] || '';
            panel.text = `<div><h3 style=\"margin:0 0 8px 0;\">${label}</h3>${params}</div>`;
        """,
    )
    cds.selected.js_on_change("indices", callback)

    p.renderers.append(g_renderer)

    # Show composed layout
    show(row(p, info))


def _object_params_with_properties(payload: Any) -> Optional[Dict[str, Any]]:
    """
    Build a parameter dictionary from a node payload by combining:
    - public attributes from payload.__dict__ (if present)
    - readable @property attributes defined on the payload's class
    - if payload is a dict, return it (excluding 'label')

    Private attributes (starting with '_') and the key 'label' are excluded.
    Values that raise on access are skipped. Callables are skipped.
    """
    # If the payload is already a dict, filter and return it.
    if isinstance(payload, dict):
        return {k: v for k, v in payload.items() if k != "label"}

    if payload is None:
        return None

    params: Dict[str, Any] = {}

    # Collect from __dict__ if available
    try:
        if hasattr(payload, "__dict__") and isinstance(getattr(payload, "__dict__", None), dict):
            for k, v in vars(payload).items():
                if k.startswith("_") or k == "label":
                    continue
                # Avoid adding callables
                try:
                    is_callable = callable(v)
                except Exception:
                    is_callable = False
                if not is_callable:
                    params[k] = v
    except Exception:
        pass

    # Collect readable @property attributes on the class
    try:
        import inspect

        cls = type(payload)
        for name, member in inspect.getmembers(cls):
            if not isinstance(member, property):
                continue
            if name.startswith("_") or name == "label":
                continue
            if name in params:
                continue  # do not overwrite explicit attributes
            # Access property value safely
            try:
                value = getattr(payload, name)
            except Exception:
                continue
            # Skip callables
            try:
                if callable(value):
                    continue
            except Exception:
                pass
            params[name] = value
    except Exception:
        # If inspection fails, just ignore properties
        pass

    return params if params else None


def _format_params(params: Optional[Dict[str, Any]]) -> str:
    """Return HTML for parameter dict suitable for the side panel."""
    if not params:
        return "<i>No parameters</i>"
    try:
        items = []
        for k, v in params.items():
            items.append(f"<tr><td style='padding-right:8px; white-space:nowrap;'><b>{k}</b></td><td>{_escape_html(v)}</td></tr>")
        return "<table>" + "".join(items) + "</table>"
    except Exception:
        return f"<pre>{_escape_html(params)}</pre>"


def _escape_html(value: Any) -> str:
    try:
        s = str(value)
    except Exception:
        s = repr(value)
    return (
        s.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
        .replace("'", "&#39;")
    )

"""Offline walkthrough contracts: host Python + pytest, no ROS, GPU or archive."""

import base64
import json
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit


PAGE = Path(__file__).resolve().parents[2] / "docs/implementation/terrain-algorithm-overview.html"


class Walkthrough(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = []
        self.links = []
        self.assets = []
        self.fixture_chunks = []
        self.in_fixture = False
        self.feed(PAGE.read_text())
        self.close()

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if "id" in attrs:
            self.ids.append(attrs["id"])
        if tag == "a":
            self.links.append(attrs["href"])
        if tag in {"img", "script"} and "src" in attrs:
            self.assets.append(attrs["src"])
        if tag == "link" and "href" in attrs:
            self.assets.append(attrs["href"])
        if tag == "script":
            self.in_fixture = attrs.get("id") == "fixture-data"

    def handle_endtag(self, tag):
        if tag == "script":
            self.in_fixture = False

    def handle_data(self, data):
        if self.in_fixture:
            self.fixture_chunks.append(data)


def test_local_links_and_unique_anchors():
    page = Walkthrough()
    assert len(page.ids) == len(set(page.ids))
    for link in page.links:
        parsed = urlsplit(link)
        assert not parsed.scheme and not parsed.netloc, link
        if parsed.path:
            assert (PAGE.parent / unquote(parsed.path)).is_file(), link
        else:
            assert parsed.fragment in page.ids, link


def test_figure_embedded_without_remote_assets():
    page = Walkthrough()
    assert len(page.assets) == 1
    prefix = "data:image/png;base64,"
    assert page.assets[0].startswith(prefix)
    png = base64.b64decode(page.assets[0][len(prefix):], validate=True)
    assert png.startswith(b"\x89PNG\r\n\x1a\n")
    assert len(png) == 59446
    assert "RECORDED_FIGURE" not in PAGE.read_text()


def test_frozen_failure_fixture_provenance_and_map_cells():
    data = json.loads("".join(Walkthrough().fixture_chunks))
    assert not data.get("pending")
    assert data["provenance"]["sha256"] == {
        "parent": "69d182cadf025fa8f8c11934566fad0322f4d4d7c8f72f1f79faa14b6b34f9ff",
        "candidate": "7a13403fad68b6ab9cf445cb05fe9f4e586841ccd25de9b152c73d3f339a9c85",
    }
    assert data["resolution"] == .1
    assert set(data["slopes"]) == {"0", "-0.15", "0.15"}
    crop = data["crop"]
    fx, fy = data["failure_cell"]
    for slope in data["slopes"].values():
        for name, output in slope.items():
            assert output["false_free_probes"] == (0 if name == "parent" else 1)
            assert len(output["cells"]) == crop["height"]
            for row in output["cells"]:
                assert len(row) == crop["width"]
                for cost, reason in row:
                    assert cost in {-1, 0, 20, 100}
                    assert reason in range(13)
            cost = output["cells"][fy - crop["y"]][fx - crop["x"]][0]
            assert (cost in {0, 20}) == (name == "candidate")
    # Geometry and wall probe are evaluator overlays, never fabricated returns.
    px, py = data["probe"]
    ox, oy = data["origin"]
    assert ox + fx * .1 <= px < ox + (fx + 1) * .1
    assert oy + fy * .1 <= py < oy + (fy + 1) * .1
    # NumPy arange in the frozen fixture includes the near-0.9 endpoint.
    assert len(data["wall_returns"]) == 51

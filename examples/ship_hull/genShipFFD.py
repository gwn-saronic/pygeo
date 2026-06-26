"""
Generate an FFD control box for the KCS container-ship half-hull
(``KCS_hull_SVA.igs``).

The symmetry plane is at y=0, x runs longitudinally (stern -> bow) and z is
vertical (keel -> deck).

Two constructions are available, selected by ``BODY_FITTED``:

* Body-fitted (default): at every longitudinal station the outer transverse face
  hugs the hull's local half-beam ``y(x, z)`` and the keel/deck levels follow the
  hull, so the box tapers naturally toward the bow and stern. This places each
  control point close to the surface it influences, giving much tighter local
  shape control. The shape is measured by ray casting: the hull IGES is
  triangulated and inward ``-y`` rays are cast from outboard, keeping the
  outermost skin intersection (:func:`pygeo.geo_utils.createFittedHullFFD`).

* Rectangular box: a single axis-aligned volume sized to the hull bounding box
  plus a margin on every face, with the longitudinal sections cosine-clustered
  toward the bow and stern.

Either way the ``j=0`` control plane is pinned on the y=0 centreline to preserve
port/starboard symmetry, and the FFD index convention is i -> longitudinal,
j -> transverse (y, j=0 on the centreline), k -> vertical (z), matching what
``runShipFFD.py`` expects.

Run this once to (re)generate ``KCS_ffd.xyz``.
"""

# External modules
import numpy as np

# First party modules
from pygeo import pyGeo
from pygeo.geo_utils import createFittedHullFFD, write_wing_FFD_file

IGES_FILE = "KCS_hull_SVA.igs"

# Switch between the body-fitted FFD (True) and the simple rectangular box (False).
BODY_FITTED = True

# Number of control points in each FFD direction.
# i -> longitudinal, j -> transverse (y), k -> vertical (z).
N_LONGITUDINAL = 22
N_TRANSVERSE = 6
N_VERTICAL = 8

# --- Body-fitted parameters ------------------------------------------------- #
# Margins that grow the FFD outward so it fully encloses the hull, given as
# [longitudinal, transverse, vertical]. Absolute margins are in metres; relative
# margins are fractions of hull length, local half-beam, and hull depth. The
# transverse inner face stays on the y=0 centreline regardless of these.
ABS_MARGINS = [1.0, 5.0, 2.0]
REL_MARGINS = [0.01, 0.1, 0.1]

# --- Rectangular-box parameters --------------------------------------------- #
# FFD box extents: hull bounding box plus margins (full-scale metres).
# The inner transverse face sits exactly on the centreline plane (y=0).
X_MIN, X_MAX = -9.0, 240.0  # longitudinal (stern -> bow)
Y_MIN, Y_MAX = -0.01, 17.0  # transverse (centreline -> outboard of max beam)
Z_MIN, Z_MAX = -0.5, 24.0  # vertical (below keel -> above deck)


def generate_body_fitted(fileName):
    """Write a body-fitted FFD that conforms to the hull surface."""
    geo = pyGeo(fileName=IGES_FILE, initType="iges")
    geo.doConnectivity()

    createFittedHullFFD(
        geo,
        "point-vector",
        fileName,
        nLongitudinal=N_LONGITUDINAL,
        nTransverse=N_TRANSVERSE,
        nVertical=N_VERTICAL,
        absMargins=ABS_MARGINS,
        relMargins=REL_MARGINS,
        xDist="cosine",
    )


def generate_box(fileName):
    """Write a simple rectangular FFD box around the hull bounding box."""

    # write_wing_FFD_file builds the box from two end "slices" (stern and bow),
    # each holding the four cross-section corners. The slice array is indexed
    # [slice, a, b, xyz] where a selects the transverse (y) corner and b the
    # vertical (z) corner. We march between the slices along the longitudinal
    # direction (dim 0).
    def cross_section(xStation):
        return [
            # a = 0 -> y inner (centreline)
            [[xStation, Y_MIN, Z_MIN], [xStation, Y_MIN, Z_MAX]],
            # a = 1 -> y outer
            [[xStation, Y_MAX, Z_MIN], [xStation, Y_MAX, Z_MAX]],
        ]

    slices = np.array([cross_section(X_MIN), cross_section(X_MAX)])

    # axes = ["i", "j", "k"] maps the FFD i-index to the longitudinal slice
    # direction, j to the transverse (y) corners, and k to the vertical (z)
    # corners. So getLocalIndex(0) has shape (N_LONGITUDINAL, N_TRANSVERSE,
    # N_VERTICAL) and j=0 is the centreline plane.
    axes = ["i", "j", "k"]

    # Cluster control sections toward bow and stern along the longitudinal axis.
    dist = [["cosine", "linear", "linear"]]

    write_wing_FFD_file(
        fileName,
        slices,
        N0=N_LONGITUDINAL,
        N1=N_TRANSVERSE,
        N2=N_VERTICAL,
        axes=axes,
        dist=dist,
    )


def generate(fileName="KCS_ffd.xyz"):
    """Write the KCS hull FFD box to ``fileName`` in PLOT3D format."""
    if BODY_FITTED:
        generate_body_fitted(fileName)
        kind = "body-fitted"
    else:
        generate_box(fileName)
        kind = "rectangular-box"
    print(
        f"Wrote {fileName}: {N_LONGITUDINAL} x {N_TRANSVERSE} x {N_VERTICAL} {kind} FFD control points"
    )


if __name__ == "__main__":
    generate()

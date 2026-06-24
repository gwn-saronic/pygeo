"""
Generate a free-form deformation (FFD) control box for the KCS container-ship
half-hull (``KCS_hull_SVA.igs``).

The hull is a half model: the centreline symmetry plane is at y=0, x runs
longitudinally (stern -> bow) and z is vertical (keel -> deck). The bounding box
of the IGES control points (full-scale metres) is roughly::

    x: [ -6.0, 237.8 ]   (length, ~Lpp 230 m plus bow/stern overhang)
    y: [  0.0,  16.15]   (half-beam; KCS B = 32.2 m -> half = 16.1 m)
    z: [  0.0,  23.5 ]   (keel to deck)

We wrap this in a single rectangular FFD volume with a margin on every face,
clustering the longitudinal control sections toward the bow and stern (where the
hull shape changes most) with a cosine distribution.

The inner transverse face is placed exactly at y=0 so it coincides with the
centreline symmetry plane. With this layout the j=0 control-point plane governs
the centreline; leaving it fixed during optimization keeps the hull symmetric.

Run this once to (re)generate ``KCS_ffd.xyz``; ``runShipFFD.py`` reads that file.
"""

# External modules
import numpy as np

# First party modules
from pygeo.geo_utils import write_wing_FFD_file

# FFD box extents: hull bounding box plus margins (full-scale metres).
# The inner transverse face sits exactly on the centreline plane (y=0).
X_MIN, X_MAX = -9.0, 240.0  # longitudinal (stern -> bow)
Y_MIN, Y_MAX = 0.0, 17.0  # transverse (centreline -> outboard of max beam)
Z_MIN, Z_MAX = -0.5, 24.0  # vertical (below keel -> above deck)

# Number of control points in each FFD direction.
# i -> longitudinal, j -> transverse (y), k -> vertical (z).
N_LONGITUDINAL = 18
N_TRANSVERSE = 4
N_VERTICAL = 5


def generate(fileName="KCS_ffd.xyz"):
    """Write the KCS hull FFD box to ``fileName`` in PLOT3D format."""
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
    print(f"Wrote {fileName}: {N_LONGITUDINAL} x {N_TRANSVERSE} x {N_VERTICAL} FFD control points")


if __name__ == "__main__":
    generate()

"""
Deform the KCS container-ship half-hull with a free-form deformation (FFD).

This example embeds the IGES hull (``KCS_hull_SVA.igs``) in the FFD box produced
by ``genShipFFD.py`` and applies a *local* shape change: a
bulbous-bow-style bulge that fattens the forward sections by pushing the bow
control points outboard (in the transverse +y direction).

The hull is a half model with the centreline at y=0. The FFD's j=0 control
plane lies on that centreline, so we leave it untouched and only move the
outboard (j >= 1) control points. This keeps the centreline exactly on y=0 and
preserves port/starboard symmetry.

Outputs (PLOT3D / Tecplot / IGES):
    KCS_original.dat       undeformed hull surface (for comparison)
    KCS_deformed.plt       deformed hull surface
    KCS_deformed.igs       deformed hull as CAD (IGES)
    KCS_ffd_embedded.dat   FFD control box with the embedded hull

Run ``genShipFFD.py`` first to create ``KCS_ffd.xyz``.

Note: export prints a few sub-centimetre "not projected to tolerance" warnings.
These are CAD points with a tiny negative y (numerical noise on the centreline)
being snapped onto the y=0 symmetry plane, and are harmless.
"""

# External modules
import numpy as np

# First party modules
from pygeo import DVGeometry, pyGeo

IGES_FILE = "KCS_hull_SVA.igs"
FFD_FILE = "KCS_ffd.xyz"

# Bulge definition. The bow is at high x; the design waterline is near z = 10.8 m.
# We select forward control points at/below the waterline and push them outboard.
BOW_X_MIN = 215.0  # only stations forward of this (forward ~10% of Lpp)
WATERLINE_Z_MAX = 12.0  # only control points at/below the design waterline
BULGE_OUTBOARD = 2.0  # transverse (+y) offset applied to the selected points [m]

# Surface refinement (knots inserted per direction before export). Higher values
# give smoother output surfaces at the cost of speed.
N_REFINE = 4


def main():
    # ---------------------- Load the CAD hull (IGES) ---------------------- #
    geo = pyGeo(fileName=IGES_FILE, initType="iges")
    geo.doConnectivity()

    # Save the undeformed surface for side-by-side comparison.
    geo.writeTecplot("KCS_original.dat")

    # ------------------------- Set up the FFD ----------------------------- #
    DVGeo = DVGeometry(FFD_FILE)

    # One local shape variable per control point, free to move in the transverse
    # (y) direction. This drives the beam/fullness of the hull sections.
    DVGeo.addLocalDV("localShape", lower=-3.0, upper=3.0, axis="y", scale=1.0)

    # ----------------- Select the bow control points ---------------------- #
    # getLocalIndex(0) has shape (nLongitudinal, nTransverse, nVertical); j=0 is
    # the centreline plane. The local DV array is indexed by global coefficient
    # id, so a boolean mask over the control points maps straight onto it.
    localIndex = DVGeo.getLocalIndex(0)
    coef = DVGeo.FFD.coef

    isForward = coef[:, 0] >= BOW_X_MIN
    isBelowWaterline = coef[:, 2] <= WATERLINE_Z_MAX
    selected = isForward & isBelowWaterline

    # Never move the centreline (j=0) plane, so the symmetry plane stays at y=0.
    centrelinePlane = localIndex[:, 0, :].flatten()
    selected[centrelinePlane] = False

    print(f"Bulging {selected.sum()} bow control points outboard by {BULGE_OUTBOARD} m")

    # ----------------------- Apply the deformation ------------------------ #
    shape = DVGeo.getValues()["localShape"].copy()
    shape[selected] += BULGE_OUTBOARD
    DVGeo.setDesignVars({"localShape": shape})

    # ----------------------- Write the outputs ---------------------------- #
    # updatePyGeo embeds each surface's control points, deforms them through the
    # FFD, and writes the result. Writing both formats from the same object is
    # safe (the second call reproduces the first to ~1e-13).
    DVGeo.updatePyGeo(geo, "tecplot", "KCS_deformed", nRefU=N_REFINE, nRefV=N_REFINE)
    DVGeo.updatePyGeo(geo, "iges", "KCS_deformed", nRefU=N_REFINE, nRefV=N_REFINE)

    # Dump the FFD control box together with the embedded (deformed) hull.
    DVGeo.writeTecplot("KCS_ffd_embedded.dat")

    print("Wrote KCS_original.dat, KCS_deformed.plt, KCS_deformed.igs, KCS_ffd_embedded.dat")


if __name__ == "__main__":
    main()

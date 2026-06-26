"""
Embed the IGES hull (``KCS_hull_SVA.igs``) in the FFD box produced
by ``genShipFFD.py`` and applies *local* shape change
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
BULGE_OUTBOARD = 0.5  # transverse (+y) offset applied to the selected points [m]

STERN_X_MAX = 240.0 * 0.2 # only stations aft of this

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
    isAft = coef[:, 0] <= STERN_X_MAX
    isBelowWaterline = coef[:, 2] <= WATERLINE_Z_MAX
    bowSelectedFFDs = isForward & isBelowWaterline
    sternFFDs = isAft & isBelowWaterline

    # Never move the centreline (j=0) plane, so the symmetry plane stays at y=0.
    CLFFDs = localIndex[:, 0, :].flatten()
    bowSelectedFFDs[CLFFDs] = False
    sternFFDs[CLFFDs] = False

    print(f"Bulging {bowSelectedFFDs.sum()} bow control points outboard by {BULGE_OUTBOARD} m")

    # ----------------------- Apply the deformation ------------------------ #
    iframe = 0
    shape = DVGeo.getValues()["localShape"].copy()
    DVGeo.setDesignVars({"localShape": shape})
    nframes = 10
    wave = BULGE_OUTBOARD * np.cos(np.linspace(0, np.pi, nframes))

    # --- Bow deformation ---
    print("="*30)
    print("Bow deformation")
    print("="*30)
    for ii, bump in enumerate(wave):
        print(f"bump: {bump}")
        # breakpoint()
        shape[bowSelectedFFDs] += bump
        DVGeo.setDesignVars({"localShape": shape})

        # ----------------------- Write the outputs ---------------------------- #
        # updatePyGeo embeds each surface's control points, deforms them through the
        # FFD, and writes the result. Writing both formats from the same object is
        # safe (the second call reproduces the first to ~1e-13).
        DVGeo.updatePyGeo(geo, "tecplot", f"KCS_deformed_{iframe}", nRefU=N_REFINE, nRefV=N_REFINE)
        DVGeo.updatePyGeo(geo, "iges", f"KCS_deformed_{iframe}", nRefU=N_REFINE, nRefV=N_REFINE)

        # Dump the FFD control box together with the embedded (deformed) hull.
        DVGeo.writeTecplot(f"KCS_ffd_embedded_{iframe}.dat")

        # Reset DVs
        shape[bowSelectedFFDs] -= bump
        DVGeo.setDesignVars({"localShape": shape})
        iframe += 1

    # --- Stern def ---
    print("="*30)
    print("Stern deformation")
    print("="*30)
    for ii, bump in enumerate(wave):
        print(f"bump: {bump}")
        shape[sternFFDs] += bump
        DVGeo.setDesignVars({"localShape": shape})

        # ----------------------- Write the outputs ---------------------------- #
        # updatePyGeo embeds each surface's control points, deforms them through the
        # FFD, and writes the result. Writing both formats from the same object is
        # safe (the second call reproduces the first to ~1e-13).
        DVGeo.updatePyGeo(geo, "tecplot", f"KCS_deformed_{iframe}", nRefU=N_REFINE, nRefV=N_REFINE)
        DVGeo.updatePyGeo(geo, "iges", f"KCS_deformed_{iframe}", nRefU=N_REFINE, nRefV=N_REFINE)

        # Dump the FFD control box together with the embedded (deformed) hull.
        DVGeo.writeTecplot(f"KCS_ffd_embedded_{iframe}.dat")

        # Reset DVs
        shape[sternFFDs] -= bump
        DVGeo.setDesignVars({"localShape": shape})
        iframe += 1


if __name__ == "__main__":
    main()

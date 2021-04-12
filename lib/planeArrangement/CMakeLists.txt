# Plane Arrangement with its LP optimizer
add_library(PlaneArrangement STATIC
        PlaneArrangement.cpp PlaneArrangement.h
        PolyhedronBuilder.h
        ArrangementToMosek.cpp ArrangementToMosek.h)
target_link_libraries(PlaneArrangement libmosek64.so libfusion64.so PlaneArr)

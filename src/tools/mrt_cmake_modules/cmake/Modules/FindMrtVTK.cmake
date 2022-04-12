# find package component
if(MrtVTK_FIND_REQUIRED)
    find_package(VTK REQUIRED)
elseif(MrtVTK_FIND_QUIETLY)
    find_package(VTK QUIET)
else()
    find_package(VTK)
endif()

# Wrap all VTK_DEFINITIONS so that they are not used in CUDA code. This will prevent
# warnings generated by nvcc.
set(NEW_VTK_DEFINITION)
foreach(VTK_DEFINITION ${VTK_DEFINITIONS})
    list(APPEND NEW_VTK_DEFINITION "$<$<NOT:$<COMPILE_LANGUAGE:CUDA>>:${VTK_DEFINITION}>")
endforeach()
set(VTK_DEFINITIONS ${NEW_VTK_DEFINITION})
unset(NEW_VTK_DEFINITION)

include(${VTK_USE_FILE})
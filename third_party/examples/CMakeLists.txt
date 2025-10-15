# cmake -DKALMANIF_EXAMPLES=ON
option(KALMANIF_EXAMPLES "Build kalmanif examples." OFF)

if (KALMANIF_EXAMPLES)
  add_subdirectory(kalmanif)
endif()

# cmake -DMANIF_EXAMPLES=ON
option(MANIF_EXAMPLES "Build manif examples." OFF)

if (MANIF_EXAMPLES)
  add_subdirectory(manif)
endif()


foreach(name chassis motors remote_input hi91_imu)
  if(name STREQUAL "motors")
    set(namespace platform::motors_params)
  else()
    set(namespace modules::${name}_params)
  endif()
  set(input "${PROJECT_SOURCE_DIR}/src/chassis/defaults/${name}_params.yaml")
  set(schema "${PROJECT_SOURCE_DIR}/src/chassis/defaults/${name}_params.schema.yaml")
  set(output "${DUST_GENERATED_DIR}/${name}_params_generated.h")
  add_custom_command(OUTPUT "${output}"
    COMMAND ${Python3_EXECUTABLE} "${PROJECT_SOURCE_DIR}/tools/generate_params.py"
      --input "${input}" --schema "${schema}" --namespace "${namespace}" --output "${output}"
    DEPENDS "${PROJECT_SOURCE_DIR}/tools/generate_params.py" "${input}" "${schema}"
    VERBATIM)
  list(APPEND DUST_PARAMETER_HEADERS "${output}")
endforeach()

add_custom_target(dust_parameters DEPENDS ${DUST_PARAMETER_HEADERS})

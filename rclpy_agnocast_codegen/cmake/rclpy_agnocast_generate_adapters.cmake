# Generate one provider per companion package. Only installed interface packages
# are inputs, so ROS headers and IDL are selected from the same CMake prefix.
function(rclpy_agnocast_generate_adapters target)
  if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_CURRENT_BINARY_DIR)
    message(FATAL_ERROR "Adapter generation requires an out-of-source build")
  endif()
  cmake_parse_arguments(ARG "" "" "TYPES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS OR NOT ARG_TYPES)
    message(FATAL_ERROR "rclpy_agnocast_generate_adapters requires TYPES")
  endif()
  get_property(provider_set GLOBAL PROPERTY "_RCLPY_AGNOCAST_PROVIDER_${PROJECT_NAME}" SET)
  if(provider_set)
    message(FATAL_ERROR "Only one adapter provider per package is supported: ${PROJECT_NAME}")
  endif()
  set_property(GLOBAL PROPERTY "_RCLPY_AGNOCAST_PROVIDER_${PROJECT_NAME}" "${target}")
  find_package(rclpy_agnocast REQUIRED)
  find_package(agnocastlib REQUIRED)
  _rclpy_agnocast_selected_headers_digest(agnocast_headers_digest)
  find_package(ament_cmake_python REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(pybind11_vendor REQUIRED)
  find_package(pybind11 CONFIG REQUIRED)

  set(packages)
  set(type_args)
  foreach(type IN LISTS ARG_TYPES)
    if(NOT type MATCHES "^([a-z][a-z0-9_]*)/msg/[A-Z][a-zA-Z0-9]*$")
      message(FATAL_ERROR "Unsupported adapter type: ${type}")
    endif()
    list(APPEND packages "${CMAKE_MATCH_1}")
    list(APPEND type_args --type "${type}")
  endforeach()
  list(REMOVE_DUPLICATES packages)
  set(all_packages ${packages})
  foreach(package IN LISTS packages)
    find_package(${package} REQUIRED)
    list(APPEND all_packages ${${package}_RECURSIVE_DEPENDENCIES})
  endforeach()
  list(REMOVE_DUPLICATES all_packages)
  set(package_args)
  set(idl_dependencies)
  foreach(package IN LISTS all_packages)
    find_package(${package} REQUIRED)
    if(DEFINED ${package}_IDL_FILES)
      get_filename_component(share "${${package}_DIR}/.." ABSOLUTE)
      list(APPEND package_args --package "${package}=${share}")
      foreach(idl IN LISTS ${package}_IDL_FILES)
        list(APPEND idl_dependencies "${share}/${idl}")
      endforeach()
    endif()
  endforeach()

  set(output "${CMAKE_CURRENT_BINARY_DIR}/${target}")
  file(MAKE_DIRECTORY "${output}")
  set(resource_dir "${rclpy_agnocast_codegen_DIR}/../resource")
  add_executable(${target}_abi_probe "${resource_dir}/abi_probe.cpp")
  target_link_libraries(${target}_abi_probe PRIVATE rclpy_agnocast::rclpy_agnocast_types)
  target_compile_definitions(${target}_abi_probe PRIVATE
    RCLPY_AGNOCAST_SELECTED_AGNOCAST_HEADERS_SHA256="${agnocast_headers_digest}")
  # This baseline tag is checked before import. Module-specific settings are
  # checked by the extension's C handshake before any C++ registration call.
  set(generator_dir "${rclpy_agnocast_codegen_DIR}/../generator")
  file(GLOB generator_sources "${generator_dir}/rclpy_agnocast_codegen/*.py")
  ament_get_python_install_dir(python_install_dir)
  pybind11_add_module(${target} MODULE "${output}/provider.cpp")
  target_compile_definitions(${target} PRIVATE
    RCLPY_AGNOCAST_SELECTED_AGNOCAST_HEADERS_SHA256="${agnocast_headers_digest}")
  get_target_property(module_suffix ${target} SUFFIX)
  set(library_relative "${python_install_dir}/${PROJECT_NAME}/${target}${module_suffix}")
  # Makefiles do not rebuild outputs when only the recipe's arguments change.
  file(GENERATE OUTPUT "${output}/arguments.txt" CONTENT
    "${ARG_TYPES}\n${package_args}\n${PROJECT_NAME}\n${target}\n${library_relative}\n")
  add_custom_command(
    OUTPUT "${output}/provider.cpp" "${output}/manifest.json"
    COMMAND $<TARGET_FILE:${target}_abi_probe> "${output}/abi.txt"
    COMMAND ${Python3_EXECUTABLE} "${generator_dir}"
      --provider "${PROJECT_NAME}" --module "${PROJECT_NAME}.${target}"
      ${type_args} ${package_args} --abi-file "${output}/abi.txt" --output-dir "${output}"
      --library-relative "${library_relative}"
    DEPENDS ${target}_abi_probe ${idl_dependencies} ${generator_sources}
      "${generator_dir}/__main__.py" "${output}/arguments.txt"
    VERBATIM
  )
  target_link_libraries(${target} PRIVATE rclpy_agnocast::rclpy_agnocast_types)
  foreach(package IN LISTS packages)
    target_link_libraries(${target} PRIVATE ${${package}_TARGETS__rosidl_generator_cpp})
  endforeach()
  install(TARGETS ${target} LIBRARY DESTINATION "${python_install_dir}/${PROJECT_NAME}")
  install(FILES "${output}/manifest.json" DESTINATION "share/${PROJECT_NAME}/adapters")
  ament_index_register_resource(rclpy_agnocast_adapters
    CONTENT "share/${PROJECT_NAME}/adapters/manifest.json")
endfunction()

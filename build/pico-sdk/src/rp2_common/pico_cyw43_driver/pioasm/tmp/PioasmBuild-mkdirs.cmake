# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/jose/pico/pico-sdk/tools/pioasm"
  "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/pioasm"
  "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm"
  "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp"
  "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
  "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src"
  "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()

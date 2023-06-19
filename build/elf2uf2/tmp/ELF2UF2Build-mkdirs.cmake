# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/jose/pico/pico-sdk/tools/elf2uf2"
  "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/elf2uf2"
  "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/elf2uf2"
  "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/elf2uf2/tmp"
  "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/elf2uf2/src/ELF2UF2Build-stamp"
  "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/elf2uf2/src"
  "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/elf2uf2/src/ELF2UF2Build-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/elf2uf2/src/ELF2UF2Build-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/media/jose/BAA21AD5A21A95CB/Documentos/TEC/10semestre/embebidos/Proyecto_Final/build/elf2uf2/src/ELF2UF2Build-stamp${cfgdir}") # cfgdir has leading slash
endif()

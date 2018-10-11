
fips_begin_app( serdp_recorder cmdline )
  fips_files( SerdpRecorder.cpp )

  fips_deps( blackmagic oculus active_object videoencoder serdprecorder )
  fips_libs( avutil avcodec swscale avformat )


  fips_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

fips_end_app()

fips_begin_app( oculus_gpmf_recorder cmdline )
  fips_files( oculus_gpmf_recorder.cpp )

  fips_deps( oculus serdprecorder )

  fips_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

fips_end_app()


fips_begin_app( gpmf_parser cmdline )
  fips_files( gpmf_parser.cpp GPMF_print.c )

  fips_deps( gpmf g3logger )

  fips_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

fips_end_app()

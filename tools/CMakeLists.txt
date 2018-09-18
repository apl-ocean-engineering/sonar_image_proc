
fips_begin_app( serdp_recorder cmdline )
  fips_files(
    SerdpRecorder.cpp
  )

  fips_deps( blackmagic oculus active_object videoencoder )
  fips_libs( avutil avcodec swscale avformat )


  fips_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

fips_end_app()

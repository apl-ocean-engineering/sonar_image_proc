
fips_begin_app( serdp_recorder cmdline )
  fips_src( . )

  fips_deps( blackmagic oculus active_object videoencoder serdprecorder )
  fips_libs( avutil avcodec swscale avformat )


  fips_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

fips_end_app()

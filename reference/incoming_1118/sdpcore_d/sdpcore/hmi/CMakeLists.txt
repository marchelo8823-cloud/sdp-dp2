# Пульт оператора (Qt 6 Quick). Собирается, если в корневом CMake включено SDP_BUILD_HMI.
find_package(Qt6 REQUIRED COMPONENTS Gui Quick QuickControls2)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
qt_add_executable(sdp_hmi main.cpp hmicore.cpp hmicore.h hmi.qrc)
target_link_libraries(sdp_hmi PRIVATE dpcore Qt6::Gui Qt6::Quick Qt6::QuickControls2)
set_target_properties(sdp_hmi PROPERTIES MACOSX_BUNDLE OFF WIN32_EXECUTABLE ON)
# проверка без экрана: пульт загружается без ошибок QML и делает снимок в режиме удержания
if(NOT WIN32)
  add_test(NAME hmi_smoke COMMAND ${CMAKE_COMMAND} -E env QT_QPA_PLATFORM=offscreen QT_QUICK_BACKEND=software
           $<TARGET_FILE:sdp_hmi> --ship ${CMAKE_SOURCE_DIR}/data/Данные_судна_пример.xlsx --sim ${CMAKE_SOURCE_DIR}/tests/scenarios/имитатор_быстро.xlsx
           --run 20 --cmd "thr_take all" --cmd position --run 120 --screenshot ${CMAKE_BINARY_DIR}/hmi_smoke.png
           WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  set_tests_properties(hmi_smoke PROPERTIES TIMEOUT 120)
  # подтверждение смены режима (РС ч. XV 8.5.7): одно нажатие «Ожидание» режим не снимает, второе подряд — снимает
  foreach(n 1 2)
    set(pr "")
    foreach(i RANGE 1 ${n})
      list(APPEND pr --press "Ожидание")
    endforeach()
    if(n EQUAL 1)
      set(em 4)
    else()
      set(em 0)
    endif()
    add_test(NAME hmi_confirm_${n} COMMAND ${CMAKE_COMMAND} -E env QT_QPA_PLATFORM=offscreen QT_QUICK_BACKEND=software
             $<TARGET_FILE:sdp_hmi> --ship ${CMAKE_SOURCE_DIR}/data/Данные_судна_пример.xlsx --sim ${CMAKE_SOURCE_DIR}/tests/scenarios/имитатор_быстро.xlsx
             --run 20 --cmd "thr_take all" --cmd position --run 30 ${pr} --expect-mode ${em}
             WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    set_tests_properties(hmi_confirm_${n} PROPERTIES TIMEOUT 120)
  endforeach()
  # анализ последствий: кнопка «Класс 2» на пульте включает анализ
  add_test(NAME hmi_ca COMMAND ${CMAKE_COMMAND} -E env QT_QPA_PLATFORM=offscreen QT_QUICK_BACKEND=software
           $<TARGET_FILE:sdp_hmi> --ship ${CMAKE_SOURCE_DIR}/data/Данные_судна_пример.xlsx --sim ${CMAKE_SOURCE_DIR}/tests/scenarios/имитатор_быстро.xlsx
           --run 20 --cmd "thr_take all" --cmd position --run 30 --press "Класс 2" --expect-ca 2
           WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  set_tests_properties(hmi_ca PROPERTIES TIMEOUT 120)
endif()

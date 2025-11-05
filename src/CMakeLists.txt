add_executable(projeto_tcc_monitoramento
    main.c
    perifericos.c 
    tarefas.c 

    oled_display.c
    ssd1306_text.c
    oled_context.c

    # aht10.c
    vl53l0x.c
    braco_config.c
    fpga_comms.c

    numeros_grandes.c
    numeros_medios.c
    digitos_grandes_utils.c
    digitos_medios_utils.c
    numeros_display.c

)

# Corrige a saída para build/ em vez de build/src/
set_target_properties(projeto_tcc_monitoramento PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

target_sources(projeto_tcc_monitoramento PRIVATE
    ${FREERTOS_KERNEL_PATH}/portable/MemMang/heap_4.c
)

target_include_directories(projeto_tcc_monitoramento PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../inc

)

target_link_libraries(projeto_tcc_monitoramento
    pico_stdlib 
    FreeRTOS-Kernel
    hardware_adc
    hardware_pwm
    hardware_i2c
    hardware_uart
)

pico_add_extra_outputs(projeto_tcc_monitoramento)

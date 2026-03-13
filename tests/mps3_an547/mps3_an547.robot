*** Test Cases ***
hello_world on mps3_corstone300_an547
    ${x}=                       Execute Command         include @${CURDIR}/mps3_an547.resc
    Create Terminal Tester      sysbus.uart0    timeout=5    defaultPauseEmulation=true

    Register Failing Uart String    ZEPHYR FATAL ERROR
	
    Wait For Prompt On Uart	>
    Write Line To Uart		help
    Wait For Line On Uart       BlueOS kernel shell commands:

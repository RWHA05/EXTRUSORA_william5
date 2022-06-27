# EXTRUSORA_william5
Extrusora de filamento 3D   *****DIY*****

# DALE CLICK A LA CARPETA src->main.c PARA LEER EL CODIGO


### Instalación 🔧
Utilizé VSCode y Platformio para cargar el proyecto en la ESP32.
 El firmware es el ESP-IDF y se corre en una board = esp32dev "Es la tarjeta clásica que tiene dos leds, uno de power y otro que se controla con el pin 2".
 Puedes ver videos de vscode y platformio para ESP32 en youtube por si gustas cargar el código y aprender un poco. 
 VERIFICA QUE TUS COMPONENTES SE ADAPTEN AL CODIGO, POR EJEMPLO EL ENCODER O LA PANTALLA LCD, ETC.
 
PUEDES GENERAR UN NUEVO PROYECTO EN PLATFORMIO ELIGIENDO EL FIRMWARE ESP-IDF Y LA TARJETA CORRESPONDIENTE, TE VA A GENERAR UNA CARPETA CON VARIOS ARCHIVOS, EN LA CARPETA src PEGA LOS ARCHIVOS .c Y .h DE ESTE REPOSITORIO, COMPILA Y ESO ES TODO. NOTA: PUEDE QUE NO COMPILE SI LA VERSION DEL FIRMWARE DE ESP-IDF YA ACTUALIZO ALGUNA DE LAS FUNCIONES QUE SE OCUPAN AQUI, TENDRIAS QUE ACTUALIZAR MANUALMENTE LAS FUNCIONES QUE TE MARQUEN ERROR SI ES EL CASO.

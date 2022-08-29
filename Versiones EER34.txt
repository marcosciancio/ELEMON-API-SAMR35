EER34 API para SAMR34 y SAMR35:
------------------------------


Nuevas versiones al 5/7/2022
----------------------------
* SAMR34:
  API_7_3

* SAMR35
  API_7_3 (R35)


Versiones API_7_3 y API_7_3 (R35)
Derivadas de la version 7_2 y conservando la misma version del stack LoRaWAN, incluye las funciones para operar en modo radio.
Tiene unos defines en app.c para compilar el tipo de demo de modo radio.

Nuevas versiones al 17/12/2021
------------------------------
* SAMR34:
  API_6_1
  API_7_1
  API_7_2
  API_8
  API_8_1
	
* SAMR35
  API_7_1 (R35)
  API_7_2 (R35)
  API_8 (R35)
  API_8_1 (R35)
	
Version API_6_1:
  Solo para SAMR34.
  Basada en API_6, sin cambios en el codigo fuente.
	Solo se reoordenaron los archivos fuentes agregando la carpeta Drivers.
	Usa el stack LoRaWAN MLS_SDK_1_0_P_3.

Versiones API_7_1 y API_7_1 (R35)
  Para SAMR34 y SAMR35 respectivamente, identicas entre si, solo cambia
  el dispositivo.
  Basada en API_7 (R35), sin cambios en el codigo fuente.
	Solo se reoordenaron los archivos fuentes agregando la carpeta Drivers.
	Usan el stack LoRaWAN MLS_SDK_1_0_P_3.
 
Versiones API_8 y API_8 (R35)
  Para SAMR34 y SAMR35 respectivamente, identicas entre si, solo cambia
  el dispositivo.
	Usan el stack LoRaWAN MLS_SDK_1_0_P_5, compatible con LoRaWAN 1.0.4 pero no
	con LoRaWAN 1.0.2.

Versiones API_7_2 y API_7_2 (R35)
  En EER34.c Se corrigio un bug en appDataCallback() por el cual reportaba mal
	el evento EER34_STATUS_TX_SUCCSESS, y se agrego en EER34_status_t el valor
	EER34_STATUS_TX_ERROR, que por ejemplo lo da cuando no recibe el ACK en una
	transmision con confirmacion. Tambien se agrando el timeout default para que
	no de tiemaout si todavia esta intentando transmitir con confirmacion.
	Se modifico APP.c para que interprete el nuevo status de error de 
	transmision.
	Se hizo que la funcion EER34_setBand() se implemento la subbanda 0 que 
	habilita todos los canales.

Versiones API_8_1 y API_8_1 (R35)
	Tiene las mismas correciones y cambios que la API_7_2.
	
Versiones API_8_2 y API_8_2 (R35)
	Incluye los archivos de manejo de perifericos, SPI, I2C, UARTx2, GPIO, ADC, etc
	
Versiones API_8_3 y API_8_3 (R35)
	Incluye los archivos para el control del acelerometro y memoria eeprom_64


Versiones estables anteriores
-----------------------------
Versiones disponibles en la Web de Elemon al 10/12/2021:

* SAMR34:
  API_6
	
* SAMR35:
  API_7 (R35)
	
Ambas versiones usan el stack LoRaWAN MLS_SDK_1_0_P_3.

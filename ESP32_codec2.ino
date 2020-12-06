/*
 *********************    Codec2 in ESP32    ********************

 This test program implement the encoder and decoder of Codec2
 at 1600bps using LoRa radio.

 Codec 2 is a low-bitrate speech audio codec (speech coding) 
 that is patent free and open source develop by David Grant Rowe.
 http://www.rowetel.com/
 
 This program samples the audio in the transmitter at 8KHz using 
 an ADC a reproduces it in the receiver using a DAC.

 Every 40ms will be generate a new codec2 encoded frame with 8 bytes, 
 then every 5 codec2 frames will be generate a transmission frame.
 In this schema a transmission happened at 200ms intervals, so you 
 have less than 200ms to make the transmission (I'm using 182ms).

 In this implementation the transmission frame has 44 bytes, the 
 first 4 bytes are the header and the others are the voice.
 You can use the header to indicate the address of the transmitter, 
 the address of the desire receiver, etc.

 

 ***********************   W A R N I N G   ********************
 
 This test program DO NOT complies with FCC regulations from USA 
 even not complies with ANATEL from Brazil and I guess do not 
 complies with any regulations from any country.

 To complies with the FCC and orders foreign agencies, normally 
 you need to implement a frequency hopping algorithm that is 
 outside the scope of this test program.

 Please verify your country regulations. You assume all 
 responsibility for the use or misuse of this test program.

 TIP: 
 The challenge of a frequency hopping system is the synchronization, 
 maybe you can use a GPS receiver for synchronization.

 **************************************************************

*/

const char * ssid = "YOUR_SSID";
const char * password = "YOUR_PASS";




#include <Arduino.h>
#include <driver/adc.h>
#include "esp32-hal-cpu.h"

#include "WiFi.h"
#include "AsyncUDP.h"
AsyncUDP udp;

#define MODE3200
#include <codec2.h>				//In the codec2 folder in the library folder
#include <ButterworthFilter.h>	//In the codec2 folder in the library folder
#include <FastAudioFIFO.h>		//In the codec2 folder in the library folder

#define ADC_PIN ADC1_CHANNEL_7 //ADC 1 canal 07 GPIO35  ADC1:36,37,38,39,32,33,34,35
#define DAC_PIN 25      //25 or 26
#define PTT_PIN 37
#define LED 13

#ifdef MODE3200
  #define ADC_BUFFER_SIZE 160 //20ms of voice in 8KHz sampling frequency
#else
  #define ADC_BUFFER_SIZE 320 //40ms of voice in 8KHz sampling frequency
#endif
#define ENCODE_FRAME_SIZE 16
#define ENCODE_CODEC2_FRAME_SIZE 8
#define CRC_POLY_16 0x5935u
#define CRC_START_16 0xFFFFu

IPAddress serverIP= IPAddress(3,138,122,152);     //M17-USA


uint32_t localPort = 17000;      // standart m17 port
char packetBuffer[70]; //buffer to hold incoming packet, 54 bytes actually

char base40address[6];
char destid[6];

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete


FastAudioFIFO audio_fifo;

enum RadioState
{
	radio_standby, radio_rx, radio_tx 
};
volatile RadioState radio_state = RadioState::radio_tx;

int16_t adc_buffer[ADC_BUFFER_SIZE];
int16_t speech[ADC_BUFFER_SIZE];
int16_t output_buffer[ADC_BUFFER_SIZE];
uint8_t transmitBuffer[ADC_BUFFER_SIZE];
unsigned char rx_encode_frame[ENCODE_FRAME_SIZE];
int adc_buffer_index = 0;
unsigned char tx_encode_frame[ENCODE_FRAME_SIZE];
int tx_encode_frame_index = 0;
  
//The codec2 
struct CODEC2* codec2_state;

//Implement a high pass 240Hz Butterworth Filter.
ButterworthFilter hp_filter(240, 8000, ButterworthFilter::ButterworthFilter::Highpass, 1);

hw_timer_t* adcTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t codec2HandlerTask;

long last_tick = 0;
long slapsed_in = 0;
uint8_t rx_raw_audio_value = 127;

volatile bool rx_ok = false;
volatile bool tx_ok = false;

int slapsed_encoder, slapsed_decoder, slapsed_tx, slapsed_ack, slapsed_rx, slapsed_rx_ack;
long start_tx;


char * uintToStr( const uint64_t num, char *str )
{
  uint8_t i = 0;
  uint64_t n = num;
 
  do
    i++;
  while ( n /= 10 );
 
  str[i] = '\0';
  n = num;
 
  do
    str[--i] = ( n % 10 ) + '0';
  while ( n /= 10 );

  return str;
}

void encode_callsign_base40(const char *callsign) {
  uint64_t encoded = 0;
  for (const char *p = (callsign + strlen(callsign) - 1); p >= callsign; p-- ) {
    encoded *= 40;
    // If speed is more important than code space, you can replace this with a lookup into a 256 byte array.
    if (*p >= 'A' && *p <= 'Z') // 1-26
      encoded += *p - 'A' + 1;
    else if (*p >= '0' && *p <= '9') // 27-36
      encoded += *p - '0' + 27;
    else if (*p == '-') // 37
      encoded += 37;
    // These are just place holders. If other characters make more sense, change these.
    // Be sure to change them in the decode array below too.
    else if (*p == '/') // 38
      encoded += 38;
    else if (*p == '.') // 39
      encoded += 39;
    //else
      // Invalid character or space, represented by 0, decoded as a space.
      //encoded += 0;
  }
  //return encoded;
  //Serial.println("Encoded:");
  //char str[21];
  //Serial.println(uintToStr( encoded, str ));

  for(int i=5;i>0;i--)
  {   base40address[i] = encoded & 0xFF;
      encoded = encoded >> 8;
  }
  base40address[0] = encoded & 0xFF;
}

char *decode_callsign_base40(uint64_t encoded, char *callsign) {
  if (encoded >= 262144000000000) { // 40^9
    *callsign = 0;
    return callsign;
  }
  char *p = callsign;
  for (; encoded > 0; p++) {
    *p = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-/."[encoded % 40];
    encoded /= 40;
  }
  *p = 0;

  return callsign;
}

void refl_conn(char module)
{
    unsigned char mesg[11];//={'C','O','N','N',base40address[5],base40address[4],base40address[3],base40address[2],base40address[1],base40address[0],module};
    memcpy(mesg,"CONN",4);
    memcpy(mesg+4,base40address,6);
    mesg[10] = module;
    
    //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Serial.print("Connecting to reflector, module: ");
    Serial.println(module);
    //Serial.print("Mesg: "); Serial.println(mesg);
    //Udp.beginPacket(reflectorIP, localPort);
    udp.write(mesg,sizeof(mesg));
    //Udp.endPacket();  
}

void refl_ackn()
{
    unsigned char mesg[10];//={'A','C','K','N',base40address[5],base40address[4],base40address[3],base40address[2],base40address[1],base40address[0]};
    memcpy(mesg,"ACKN",4);
    memcpy(mesg+4,base40address,6);
    Serial.println("Sending ACKN");
    //Udp.beginPacket(reflectorIP, localPort);
    udp.write(mesg,sizeof(mesg));
    //Udp.endPacket();   
}

void refl_nack()
{
    unsigned char mesg[4]={'N','A','C','K'};
    Serial.println("Sending NACK");    
    //Udp.beginPacket(reflectorIP, localPort);
    udp.write(mesg,sizeof(mesg));
    //Udp.endPacket(); 
}

void refl_pong()
{
    unsigned char mesg[10];//={'P','O','N','G',base40address[5],base40address[4],base40address[3],base40address[2],base40address[1],base40address[0]};
    memcpy(mesg,"PONG",4);
    memcpy(mesg+4,base40address,6);
    
    Serial.print(".");
    //Serial.print("Mesg: "); Serial.println(mesg);    
    //Udp.beginPacket(reflectorIP, localPort);
    udp.write(mesg,sizeof(mesg));
    //Udp.endPacket();
}

void refl_disc()
{
    unsigned char mesg[10];//={'D','I','S','C',base40address[5],base40address[4],base40address[3],base40address[2],base40address[1],base40address[0]};
    memcpy(mesg,"DISC",4);
    memcpy(mesg+4,base40address,6);
    
    Serial.println("Sending DISC");    
    //Udp.beginPacket(reflectorIP, localPort);
    udp.write(mesg,sizeof(mesg));
    //Udp.endPacket();
}

unsigned char FN[2], SID[2];   //frame Number, StreamID

void refl_stdframe()
{
  uint16_t crc_tab16[256];
  unsigned char  NONCE[14]={0},CRC_16[2] ={0x00,0x00}, TYPE[2]={0x00,0x05};
  int p;
   
   
   unsigned char mesg[55]; 		//54  
   
   memcpy(mesg,"M17 ",4);         //Magic
   memcpy(mesg+4,SID,2);  p=6; //Stream ID  
   //LICH-(28 Byte)--------------------
   memcpy(mesg+p,destid,6);  p+=6;       //DST
   memcpy(mesg+p,base40address,6);  p+=6;       //SRC
   memcpy(mesg+p,TYPE,2);  p+=2;       //Information about the incoming data stream
   memcpy(mesg+p,NONCE,14);  p+=14;       //Nonce for encryption       
   //-------------------------
   memcpy(mesg+p,FN,2);   p+=2;        //frame Number
   FN[0]++; if(FN[0] == 0) FN[1]++;		//TODO: 0x8000 for EOT

   memcpy(mesg+p,tx_encode_frame,ENCODE_FRAME_SIZE);  p+=ENCODE_FRAME_SIZE;        //Payload
   //TODO: calculate CRC
   memcpy(mesg+p,CRC_16,2);    p+=2;
   Serial.print("sending "); Serial.print(p); Serial.println("bytes."); 

   Serial.print("Contents:");
                for(int i=0;i<p;i++)
                  {//Serial.print("{0x");
                       if(mesg[i]<0x10) Serial.print("0");
                       Serial.print(mesg[i],HEX);
                       Serial.print(",");
                      
                  }

    
   udp.write(mesg,p);
}


//int16_t 1KHz sine test tone
int16_t Sine1KHz[8] = { -21210 , -30000, -21210, 0 , 21210 , 30000 , 21210, 0 };
int Sine1KHz_index = 0;

void IRAM_ATTR onTimer() {
	portENTER_CRITICAL_ISR(&timerMux); //Enter crital code without interruptions

	if (radio_state == RadioState::radio_tx)
	{
		//Read the ADC and convert is value from (0 - 4095) to (-32768 - 32767)
		adc_buffer[adc_buffer_index++] = (16 * adc1_get_raw(ADC_PIN)) - 32768;

		//If you want to test with a 1KHz tone, comment the line above and descomment the three lines below

		//adc_buffer[adc_buffer_index++] = Sine1KHz[Sine1KHz_index++];
		//if (Sine1KHz_index >= 8)
		//	Sine1KHz_index = 0;

		//When buffer is full
		if (adc_buffer_index == ADC_BUFFER_SIZE) {
			adc_buffer_index = 0;

			slapsed_in = millis() - last_tick; //Just for debug
			last_tick = millis(); //Just for debug

			//Transfer the buffer from adc_buffer to speech buffer
			memcpy((void*)speech, (void*)adc_buffer, 2 * ADC_BUFFER_SIZE); 

			// Notify run_codec2 task that the buffer is ready.
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			vTaskNotifyGiveFromISR(codec2HandlerTask, &xHigherPriorityTaskWoken);
			if (xHigherPriorityTaskWoken)
			{
				portYIELD_FROM_ISR();
			}
		}
	}
	else if (radio_state == RadioState::radio_rx)
	{
		int16_t v;

		//Get a value from audio_fifo and convert it to 0 - 255 to play it in the ADC
		//If none value is available the DAC will play the last one that was read, that's
		//why the rx_raw_audio_value variable is a global one.
		if (audio_fifo.get(&v))
			rx_raw_audio_value = (uint8_t)((v + 32768) / 256);

		//Play
		dacWrite(DAC_PIN, rx_raw_audio_value);
	}
	portEXIT_CRITICAL_ISR(&timerMux); // exit critical code
}

void run_codec2(void* parameter)
{
	//Init codec2
	#ifdef MODE3200
      codec2_state = codec2_create(CODEC2_MODE_3200);
  #else    
	    codec2_state = codec2_create(CODEC2_MODE_1600);
	#endif
	codec2_set_lpc_post_filter(codec2_state, 1, 0, 0.8, 0.2);

	long start_encoder, start_decoder; //just for debug
	
	RadioState last_state = RadioState::radio_standby;
	while (1)
	{
		//Wait until be notify or 1 second
		uint32_t tcount = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));

		if (tcount != 0) //if the task was notified! 
		{
			//Init the tx_encode_frame_index if a trasmition start
			if (radio_state != last_state)
			{
				if (radio_state == RadioState::radio_tx)
				{
					tx_encode_frame_index = 0;
          randomSeed(millis());
          SID[0] = random(0xFF);    //Create new Stream ID for each transmission
          SID[1] = random(0xFF);
				}
				last_state = radio_state;
			}

			
			if (radio_state == RadioState::radio_tx) //Trasnmitting
			{
				start_encoder = millis(); //Just for debug

				//Apply High Pass Filter
				for (int i = 0; i < ADC_BUFFER_SIZE; i++)
					speech[i] = (int16_t)hp_filter.Update((float)speech[i]);

				//encode the 320 bytes(40ms) of speech frame into 8 bytes
				codec2_encode(codec2_state, tx_encode_frame + tx_encode_frame_index, speech);	

				//increment the pointer where the encoded frame must be saved
				tx_encode_frame_index += ENCODE_CODEC2_FRAME_SIZE; 

				slapsed_encoder = millis() - start_encoder; //Just for debug

				//If it is the 5th time then we have a ready trasnmission frame
				if (tx_encode_frame_index == ENCODE_FRAME_SIZE)
				{
					start_tx = millis(); //Just for debug
					tx_encode_frame_index = 0;

					//Transmit it
          refl_stdframe();
          
          tx_ok = true;
          slapsed_tx = millis() - start_tx; //Just for debug
				}
			}
			if (radio_state == RadioState::radio_rx) //Receiving
			{
				start_decoder = millis(); //Just for debug

				//Make a cycle to get each codec2 frame from the received frame
				for (int i = 0; i < ENCODE_FRAME_SIZE; i += ENCODE_CODEC2_FRAME_SIZE)
				{
					//Decode the codec2 frame
					codec2_decode(codec2_state, output_buffer, rx_encode_frame + i);
					
					// Add to the audio buffer the 320 samples resulting of the decode of the codec2 frame.
					for (int g = 0; g < ADC_BUFFER_SIZE; g++)
						audio_fifo.put(output_buffer[g]);
				}

				slapsed_decoder = millis() - start_decoder; //Just for debug
				rx_ok = true;				
			}
		}
	}
}

void setup() {
	Serial.begin(115200); //Just for debug
  //Configure PTT input
  pinMode(PTT_PIN, INPUT_PULLUP);
  //pinMode(TEST_PIN, INPUT_PULLUP);
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  //TODO: Reduce power when idle
//setCpuFrequencyMhz(80); //Set CPU clock to 80MHz fo example
//  Serial.print("Speed: ");
//  Serial.println(getCpuFrequencyMhz());//Get CPU clock

  WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("WiFi Failed");
        while(1) {
            delay(1000);
        }
    }
  delay(5000);
  
  encode_callsign_base40("M17-USA A"); //Reflector call with Module with a space between
  memcpy(destid, base40address, 6);
  
  Serial.print("Dst: ");
  Serial.print(destid[5],HEX);   Serial.print(":");
  Serial.print(destid[4],HEX);   Serial.print(":");
  Serial.print(destid[3],HEX);   Serial.print(":");
  Serial.print(destid[2],HEX);   Serial.print(":");
  Serial.print(destid[1],HEX);   Serial.print(":");
  Serial.println(destid[0],HEX);
  
  encode_callsign_base40("YOURCALL   D");		//It has to be 9 chars
  Serial.print("Src: ");
  Serial.print(base40address[5],HEX);   Serial.print(":");
  Serial.print(base40address[4],HEX);   Serial.print(":");
  Serial.print(base40address[3],HEX);   Serial.print(":");
  Serial.print(base40address[2],HEX);   Serial.print(":");
  Serial.print(base40address[1],HEX);   Serial.print(":");
  Serial.println(base40address[0],HEX);

    if(udp.connect(serverIP, localPort)) {
        Serial.println("UDP connected");
        udp.onPacket([](AsyncUDPPacket packet) {
            /*
            //Serial.print("UDP Packet Type: ");        Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
            Serial.print(", From: ");      Serial.print(packet.remoteIP());
            //Serial.print(":");        Serial.print(packet.remotePort());
            Serial.print(", To: ");   Serial.print(packet.localIP());  
            //Serial.print(":");    Serial.print(packet.localPort());
            Serial.print(", Length: ");    Serial.print(packet.length());
            Serial.print(", Data: ");     Serial.write(packet.data(), packet.length());
            Serial.println();
*/
            memcpy(packetBuffer,packet.data(),packet.length());
            ///////////////////////////////////////////////
            if(packetBuffer[0]=='P' && packetBuffer[1]=='I' && packetBuffer[2]=='N' && packetBuffer[3]=='G')
                refl_pong();
            else if(packetBuffer[0]=='M' && packetBuffer[1]=='1' && packetBuffer[2]=='7' && packetBuffer[3]==' ')
            {
               Serial.print("Contents:");
                for(int i=0;i<packet.length();i++)
                  {if(packetBuffer[i]>0x21 && packetBuffer[i]<127)
                      Serial.print(packetBuffer[i]);
                    else
                      {Serial.print("{0x");
                       if(packetBuffer[i]<0x10) Serial.print("0");
                       Serial.print(packetBuffer[i],HEX);
                       Serial.print("}");
                      }
                  }
              
            memcpy(rx_encode_frame,packetBuffer+36,16);
            //memcpy(codecBuffer,packetBuffer+44,8);
        
            uint64_t DST = packetBuffer[6];   DST<<=8;
            DST += packetBuffer[7];   DST<<=8;
            DST += packetBuffer[8];   DST<<=8;
            DST += packetBuffer[9];   DST<<=8;
            DST += packetBuffer[10];   DST<<=8;
            DST += packetBuffer[11];

            uint64_t SRC = packetBuffer[12];   SRC<<=8;
            SRC += packetBuffer[13];   SRC<<=8;
            SRC += packetBuffer[14];   SRC<<=8;
            SRC += packetBuffer[15];   SRC<<=8;
            SRC += packetBuffer[16];   SRC<<=8;
            SRC += packetBuffer[17];

            char SRC_C[10]={};
            decode_callsign_base40(SRC,SRC_C);
            SRC_C[9]=0;
        
            char DST_C[10]={};
            decode_callsign_base40(DST,DST_C);
            DST_C[9]=0;
            Serial.print(SRC_C);
            Serial.print("  >>>  ");
            Serial.println(DST_C);


            //////////////////////////////////////////////
            //Set the state to radio_rx because we are receiving
                radio_state = RadioState::radio_rx;

            // Notify run_codec2 task that we have received a new packet.
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                vTaskNotifyGiveFromISR(codec2HandlerTask, &xHigherPriorityTaskWoken);
                if (xHigherPriorityTaskWoken)
                {
                    portYIELD_FROM_ISR();
                }
            }
    
            
        });
    }


	adc1_config_width(ADC_WIDTH_12Bit);
	adc1_config_channel_atten(ADC_PIN, ADC_ATTEN_DB_6); //ADC 1 canal 0 GPIO36

	//Start the task that run the coder and decoder
	xTaskCreate(&run_codec2, "codec2_task", 30000, NULL, 5, &codec2HandlerTask);

	//Start a timer at 8kHz to sample the ADC and play the audio on the DAC.
	adcTimer = timerBegin(3, 500, true); // 80 MHz / 500 = 160KHz MHz hardware clock
	timerAttachInterrupt(adcTimer, &onTimer, true); // Attaches the handler function to the timer 
	timerAlarmWrite(adcTimer, 20, true); // Interrupts when counter == 20, 8.000 times a second
	timerAlarmEnable(adcTimer); //Activate it

	last_tick = millis(); //Just for debug

	//Set state 
	radio_state = RadioState::radio_rx;

  refl_conn('A');			//Send connect message to reflector
}

int tx_ok_counter = 0; //Just for debug
int rx_ok_counter = 0; //Just for debug


void loop() {

	if (digitalRead(PTT_PIN) == LOW)
	{ 
		radio_state = RadioState::radio_tx;
    digitalWrite(LED,HIGH);
	}
	else if(tx_ok)
	{
		radio_state = RadioState::radio_rx;
	  digitalWrite(LED,LOW);
	}

  if (digitalRead(PTT_PIN))
      {FN[0] = 0;
       FN[1] = 0;
      }
  
 /* 
	//Some DEBUG stuffs, you can remove it if you want.
	if (rx_ok)
	{
		Serial.print(rx_ok_counter);
		Serial.print(" Dec=");
		Serial.println(slapsed_decoder);
		rx_ok_counter++;
		rx_ok = false;
	}

	if (tx_ok)
	{
		Serial.print(tx_ok_counter);
		Serial.print(" Enc=");
		Serial.print(slapsed_encoder);
		Serial.print(" Tx=");
		Serial.println(slapsed_tx);		
		tx_ok_counter++;
		tx_ok = false;
	}
*/
	delay(5);//At least 1ms please!
}
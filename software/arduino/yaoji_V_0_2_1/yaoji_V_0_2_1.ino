  /*-----------------------------------------------------------------------------
 Project      : YAOJI Digital LED Plant
 Author       : YunFei
 E-mail       : automatic.dai@gmail.com
 Website      : http://www.yfworld.com
 Version      : V0.1.0
 Last_Updated : 06/30/2013
-----------------------------------------------------------------------------*/

#include <timer.h>
#include <FastSPI_LED2.h>
#include <dht.h>
#include <Wire.h>

///////////////////////////////////////////////////////////////////////////////
// R2Clound Pin Assign
#define	ARD0	(2)
#define ARD1	(3)
#define ARD2	(4)
#define ARD3	(5)
#define ARD4	(6)
#define ARD5	(7)
#define ARD6	(8)
#define ARD7	(12)

#define ARA0	(0)
#define ARA1	(1)
#define ARA2	(2)
#define ARA3	(3)
#define ARA4	(6)
#define ARA5	(7)

#define ARPWM1	(9)
#define ARPWM2	(10)
#define ARPWM3	(11)

#define	ARLED	(13)
///////////////////////////////////////////////////////////////////////////////
dht 	DHT;
CTimer	tm_action;
CTimer	tm_sensor;
CTimer	tm_life;
///////////////////////////////////////////////////////////////////////////////
// CYaoji
#define YAOJI_STATE_IDLE		(0x00)
#define YAOJI_STATE_BUSY		(0x01)

// PINs Defination
#define TM1809_PIN				(ARD2)		// put the tm1809 signal in D4
#define DHT11_PIN               (ARD4)		// put the dht sensor in D6
#define SHOCK_PIN				(ARD5)
#define INTERACT_PIN			(ARD6)

// Macro Defination
#define YAOJI_DEBUG_ON			(0)
#define YAOJI_LED_NUMBERS 		(5)

#define PARAM_LIFE				(0x0000)
#define PARAM_IDLE_MODE			(0x0001)
#define PARAM_TEMPERATURE		(0x0002)
#define PARAM_HUMIDITY			(0x0003)
#define PARAM_PRESSURE			(0x0004)
#define PARAM_LUMIN				(0x0005)
#define PARAM_RGB1				(0x0006)
#define PARAM_RGB2				(0x0007)
#define PARAM_RGB3				(0x0008)
#define PARAM_RGB4				(0x0009)
#define PARAM_RGB5				(0x000A)
#define PARAM_RGBALL			(0x000B)
#define PARAM_RGBBURST			(0x000C)
#define PARAM_SENSOR_BURST		(0x0010)
#define PARAM_ENVIRONMENT		(0x0011)

class CYaoji
{
 // Members
 private:
	int 	state;					// 0x00 IDLE, 0x01 BUSY
	int 	idle_mode;				// 0 - 20
	int		busy_mode;				// Environment situation with BUSY
	byte	lamp_current_state;		// If lamp is on or off when in IDLE
	int 	life;					// 0 - 1,000 * 0.1%
 	int 	temperature;			// -500 - 800
	int 	humidity;				// 0 - 1,000
	long 	pressure;				// 5,000 - 106,000
    long	lux;					// 0 - 200,000
	int		shock;					// 
	int		infra1;					//
	int		infra2;					//
	int		breathe;				//
	
 public:
	struct CRGB rgbleds[YAOJI_LED_NUMBERS];			// RGB LED Array
	struct CRGB rgbleds_last[YAOJI_LED_NUMBERS];	// RGB LED Array(Last)
 // Functions
 private:
	
 public:
	CYaoji();
	~CYaoji();
	
	void 	init(void);
	void	updateSensors(void);
	void	commWithClient(void);
	void	interactFromEnv(void);
	void	decisionMake(void);
	void	takeAction(void);
    void    refreshRGB(void);
	void	selfCheck(void);
};

CYaoji::CYaoji()
{
	state = 0x00;
	idle_mode = 0x00;
	life = 1000;
	temperature = 0;
	humidity = 0;
	pressure = 0;
	lux = 0;
	memset(rgbleds, 0, YAOJI_LED_NUMBERS * sizeof(struct CRGB));
	lamp_current_state = 0;
	tm_action.setTime(100);
	tm_sensor.setTime(1000);
	tm_life.setTime(50);
}

CYaoji::~CYaoji()
{
	;
}

void CYaoji::init(void)
{
	// read config form eeprom;
	// init Serial
	serialInit(9600);
	
	// init iic
	Wire.begin();
	
	// init DHT11
	int chk = DHT.read11(DHT11_PIN);
#if (YAOJI_DEBUG_ON)
	switch (chk)
	{
		case -1: Serial.print("DHT>>Checksum error,\t"); break;
		case -2: Serial.print("DHT>>Time out error,\t"); break;
		default: Serial.print("DHT>>Unknown error,\t"); break;
	}
#endif

	// init BMP085
	// bmp085Calibration();
  
	// init BH1750
	
	// init TM1809 and LEDs
   	LEDS.setBrightness(100);
   	LEDS.addLeds<TM1809, TM1809_PIN>(rgbleds, YAOJI_LED_NUMBERS);
	memset(rgbleds, 0, YAOJI_LED_NUMBERS * sizeof(struct CRGB));
	LEDS.show();

	// init interact sensor
	pinMode(SHOCK_PIN, INPUT);
	pinMode(INTERACT_PIN, INPUT);
	
	// init system led
	pinMode(ARLED, OUTPUT);
	
}

void CYaoji::updateSensors(void)
{
	if ( tm_sensor.getReached() )
	{
		tm_sensor.resetTime();
      	// Read DHT11
      	if ( DHT.read11(DHT11_PIN) == 0 )
      	{
      		humidity = DHT.humidity * 10;
      		temperature = DHT.temperature * 10;
      	}
      	
      	// Read BMP085
      	//temperature = bmp085GetTemperature(bmp085ReadUT());
      	//pressure = bmp085GetPressure(bmp085ReadUP());
      	
      	// Sample BH1750
      	lux = BH1750_GetLux();
#if (YAOJI_DEBUG_ON)
		Serial.println(temperature);
		Serial.println(humidity);
		Serial.println(lux);
		Serial.println();
#endif
}
	
}

#define COMM_CMD_CONN			(0x0001)
#define COMM_CMD_CONNACK		(0x8001)
#define COMM_CMD_HRT			(0x0002)
#define COMM_CMD_HRTACK			(0x8002)
#define COMM_CMD_DISCON			(0x0003)
#define COMM_CMD_DISCONACK		(0x8003)
#define COMM_CMD_READREQUEST	(0x0004)
#define COMM_CMD_READACK		(0x8004)
#define COMM_CMD_WRITEREQUEST	(0x0005)
#define COMM_CMD_WRITEACK		(0x8005)

void CYaoji::commWithClient(void)
{

	serialHandle();				// get data from serial port
		
	if ( serialGetReady() )
	{
		byte *pbuff = serialGetBuff();
		int cmdLength = serialGetLength();
                
		// ack
		byte pack_ack_buff[50];
		byte *pack_ack = pack_ack_buff;

		// Param TLV
		long tag;
		long length;
		long value;
                
		// frame check
		int commandID = *(pbuff) << 8 | *(pbuff+1);
		int verifyWord = *(pbuff+cmdLength-2) << 8 | *(pbuff+cmdLength-1);
		pbuff += 2;
		cmdLength -= 4;
		
		state = YAOJI_STATE_BUSY;
		// Blink a led
		if ( digitalRead(ARLED) == HIGH )
		{
			digitalWrite(ARLED, LOW);
		}
		else
		{
			digitalWrite(ARLED, HIGH);
		}
		
		// frame analysis
		switch ( commandID )
		{
			case COMM_CMD_CONN:
			#if (YAOJI_DEBUG_ON)
				Serial.println("Client>>Conn Cmd");
			#endif			
				break;
				
			case COMM_CMD_HRT:
			#if (YAOJI_DEBUG_ON)
				Serial.println("Client>>HRT Cmd");
			#endif
				break;
				
			case COMM_CMD_DISCON:
			#if (YAOJI_DEBUG_ON)
				Serial.println("Client>>Disconn Cmd");
			#endif
				break;
				
			case COMM_CMD_READREQUEST:
			#if (YAOJI_DEBUG_ON)
				Serial.println("Client>>Read Request");
			#endif
				// TAG
				tag = *pbuff++;
				tag = tag << 8 | *pbuff++;
						
				// VALUE
				switch (tag)
				{
					// INT TYPE
					case PARAM_LIFE:
						break;
					case PARAM_IDLE_MODE:
						break;
					case PARAM_TEMPERATURE:
						break;
					case PARAM_HUMIDITY:
						break;
					case PARAM_PRESSURE:
						break;
					case PARAM_LUMIN:
						break;
					
					// BYTE TYPE
					case PARAM_RGB1:
						break;
					case PARAM_RGB2:
						break;
					case PARAM_RGB3:
						break;		
					case PARAM_RGB4:
						break;			
					case PARAM_RGB5:
						break;		
					case PARAM_RGBBURST:
						break;
					
					// Other
					case PARAM_SENSOR_BURST:
						// life
						*pack_ack++ = (byte)(life>>8); 
						*pack_ack++ = (byte)life;			
						// temperature 
						*pack_ack++ = (byte)(temperature>>8); 
						*pack_ack++ = (byte)temperature;												
						// shock(clear after read)
						*pack_ack++ = shock;
						shock = 0;
						serialSendPackage(COMM_CMD_READREQUEST, pack_ack_buff, 5);
						break;
					
					case PARAM_ENVIRONMENT:
						break;
						
					default:
						;
				}				
				break;
				
			case COMM_CMD_WRITEREQUEST:
			#if (YAOJI_DEBUG_ON)
				Serial.println("Client>>Write Request");
			#endif
				// TAG
				tag = *pbuff++;
				tag = tag << 8 | *pbuff++;
				
				// LENGTH
				length = *pbuff++;
				length = length << 8 | *pbuff++;
				
				// VALUE
				switch (tag)
				{
					// INT TYPE
					case PARAM_LIFE: 
						value = *pbuff++;
						value = value << 8 | *pbuff++;
						break;
						
					case PARAM_IDLE_MODE:
						value = *pbuff++;
						value = value << 8 | *pbuff++;
						break;
						
					// BYTE TYPE
					case PARAM_RGB1: rgbleds[0] = CRGB(*pbuff, *(pbuff+1), *(pbuff+2)); busy_mode = 0x00; break;
					case PARAM_RGB2: rgbleds[1] = CRGB(*pbuff, *(pbuff+1), *(pbuff+2)); busy_mode = 0x00; break;
					case PARAM_RGB3: rgbleds[2] = CRGB(*pbuff, *(pbuff+1), *(pbuff+2)); busy_mode = 0x00; break;	
					case PARAM_RGB4: rgbleds[3] = CRGB(*pbuff, *(pbuff+1), *(pbuff+2)); busy_mode = 0x00; break;			
					case PARAM_RGB5: rgbleds[4] = CRGB(*pbuff, *(pbuff+1), *(pbuff+2)); busy_mode = 0x00; break;	
					
					case PARAM_RGBALL:
						rgbleds[0] = CRGB(*pbuff, *(pbuff+1), *(pbuff+2)); 
						rgbleds[1] = CRGB(*pbuff, *(pbuff+1), *(pbuff+2)); 
						rgbleds[2] = CRGB(*pbuff, *(pbuff+1), *(pbuff+2)); 
						rgbleds[3] = CRGB(*pbuff, *(pbuff+1), *(pbuff+2)); 			
						rgbleds[4] = CRGB(*pbuff, *(pbuff+1), *(pbuff+2));
						busy_mode = 0x00;		// clear environment mode when user set color
						break;		
						
					case PARAM_RGBBURST:
						rgbleds[0] = CRGB(*(pbuff+0), *(pbuff+1), *(pbuff+2)); 
						rgbleds[1] = CRGB(*(pbuff+3), *(pbuff+4), *(pbuff+5));
						rgbleds[2] = CRGB(*(pbuff+6), *(pbuff+7), *(pbuff+8));
						rgbleds[3] = CRGB(*(pbuff+9), *(pbuff+10), *(pbuff+11));	
						rgbleds[4] = CRGB(*(pbuff+12), *(pbuff+13), *(pbuff+14));
						busy_mode = 0x00;		// clear environment mode when user set color
						break;
						
					case PARAM_ENVIRONMENT:
						busy_mode = *pbuff;
					#if (YAOJI_DEBUG_ON)
						Serial.print("Environment Mode :");
						Serial.println(busy_mode);
					#endif
						break;
						
					default:
						;
				}
				break;
			
			default:
				break;
		
		}
		
		// Clear ready
		serialClrReady();
	}
	
}

void CYaoji::interactFromEnv(void)
{
	// Change Yaoji's life
	if ( tm_life.getReached() )
	{
		tm_life.resetTime();
		
		if ( lux < 1000 )
		{
			if ( life != 0 )
			{
				life--;
			}
		}
		else
		{
			if ( life <= 1000 - 10 )
			{
				life += 10;
			}
		}
	}
	
	// Read Sensor
	if ( digitalRead(SHOCK_PIN) == HIGH )
	{
		shock = 0x01;
	}
	
	if ( digitalRead(INTERACT_PIN) == HIGH )
	{
		infra1 = 0x01;
	}
	else
	{
		infra1 = 0x00;
	}

}

int infra_filter;
void CYaoji::decisionMake(void)
{

    CRGB t_rgb;
	
	// Turn on or off LED on IDLE
	if ( state == YAOJI_STATE_IDLE )
	{
		if ( infra1 == 0x01 )
		{
			if ( infra_filter < 10 )
			{
				if ( ++infra_filter >= 10 )
				{
					// Off->On
					if ( lamp_current_state == 0x00 )
					{
						t_rgb = CRGB(200,200,200);
						lamp_current_state = 0x01;
						rgbleds[0] = t_rgb;
						rgbleds[1] = t_rgb;
						rgbleds[2] = t_rgb;
						rgbleds[3] = t_rgb;
						rgbleds[4] = t_rgb;
					}
					// On->Off
					else
					{
						t_rgb = CRGB(0,0,0);
						lamp_current_state = 0x00;
						rgbleds[0] = t_rgb;
						rgbleds[1] = t_rgb;
						rgbleds[2] = t_rgb;
						rgbleds[3] = t_rgb;
						rgbleds[4] = t_rgb;
					}
				}
			}
			

		}
		else
		{
			infra_filter = 0;
		}
	}
	// Environment mode when on BUSY
	else
	{
		if ( busy_mode == 0x01 )
		{
			if ( tm_action.getReached() )
			{
				tm_action.resetTime();
				if ( breathe >= 5 )
				{
					breathe -= 5;
				}
				else
				{
					breathe = 255;
				}
				memset(rgbleds, 0, YAOJI_LED_NUMBERS * sizeof(struct CRGB));
				hsv_to_rgb(breathe, 200, 200, &t_rgb);
				rgbleds[0] = t_rgb;	
				rgbleds[1] = t_rgb;
				rgbleds[2] = t_rgb;
				rgbleds[3] = t_rgb;
				rgbleds[4] = t_rgb;			
			}
		}
		else if ( busy_mode == 0x02 )
		{
			if ( tm_action.getReached() )
			{
				tm_action.resetTime();
				memset(rgbleds, 0, YAOJI_LED_NUMBERS * sizeof(struct CRGB));
				t_rgb = CRGB(random(255),random(255),random(255));
				rgbleds[0] = t_rgb;	
				rgbleds[1] = t_rgb;
				rgbleds[2] = t_rgb;
				rgbleds[3] = t_rgb;
				rgbleds[4] = t_rgb;	
			}				
		}
	}



}

void CYaoji::takeAction(void)
{
	;
}

void CYaoji::refreshRGB(void)
{
	int i;
	boolean led_changed = false;
	
	for ( i = 0; i < YAOJI_LED_NUMBERS; i++ )
	{
		if ( rgbleds_last[i] != rgbleds[i] )
		{
			// need refresh leds
			led_changed = true;
			
			// update last value
			for ( i = 0; i < YAOJI_LED_NUMBERS; i++ )
			{
				rgbleds_last[i] = rgbleds[i];
			}
			break;
		}
	}
	
	if ( led_changed )
	{
		LEDS.show();
	}
}

void CYaoji::selfCheck(void)
{
	// reset watch dog
	// LED Blink
}


///////////////////////////////////////////////////////////////////////////////
// Serial Communication
enum
{
	SERIAL_RX_IDLE      = 0,		// is idle state now, change to 
									// YAOJI_RX_STATE if get 0x55
	SERIAL_RX_START     = 1,		// wait for 0xaa
	SERIAL_RX_LENGTH_H  = 2,		// receive frame length
	SERIAL_RX_LENGTH_L  = 3,
	SERIAL_RX_BODY      = 4,		// receive command id and command body
	SERIAL_RX_WAITEND   = 5,		// wait for 0x0d
}serial_rx_state;

boolean	serial_data_ready;			// false : no frame got, true : got an available frame
int		serial_rx_cnt;
int		serial_rx_len;
int		serial_rx_buff_index;
byte	serial_rx_buff[200];

void serialInit(int baudrate)
{
	serial_data_ready = false;
	serial_rx_cnt = 0x00;
	serial_rx_state = SERIAL_RX_IDLE;
	Serial.begin(baudrate);
}

void serialHandle(void)
{
	byte byte_in;
	
	if (Serial.available() > 0)
	{
		byte_in = Serial.read();
	}
	else
	{
		return;
	}
	
	switch ( serial_rx_state )
	{
		case SERIAL_RX_IDLE:
			if ( byte_in == 0x55 )
			{
				serial_rx_state = SERIAL_RX_START;
			}
			break;
		
		case SERIAL_RX_START:
			if ( byte_in == 0xAA )
			{
				serial_rx_len = 0;
				serial_rx_state = SERIAL_RX_LENGTH_H;
			}
			else
			{
				serial_rx_state = SERIAL_RX_IDLE;
			}
			break;
		
		case SERIAL_RX_LENGTH_H:
			serial_rx_len |= byte_in;
			serial_rx_len <<= 8;
			serial_rx_state = SERIAL_RX_LENGTH_L;
			break;
			
		case SERIAL_RX_LENGTH_L:
			serial_rx_len |= byte_in;
			serial_rx_cnt = serial_rx_len;
			serial_rx_buff_index = 0;
			serial_rx_state = SERIAL_RX_BODY;
			break;
			
		case SERIAL_RX_BODY:
			serial_rx_buff[serial_rx_buff_index] = byte_in;
			serial_rx_buff_index++;
			if ( --serial_rx_cnt == 0 )
			{
				serial_rx_state = SERIAL_RX_WAITEND;
			}
			break;
			
		case SERIAL_RX_WAITEND:
			if (byte_in == 0x0D)
			{
				serial_data_ready = true;
			}
			else
			{
				serial_rx_state = SERIAL_RX_IDLE;
			}
			break;
			
		default:
			serial_rx_state = SERIAL_RX_IDLE;
	}
}

byte *serialGetBuff(void)
{
	return serial_rx_buff;
}

char serialGetLength(void)
{
	return serial_rx_len;
}

int serialGetReady(void)
{
	return serial_data_ready;
}

void serialClrReady(void)
{
	noInterrupts();
	serial_data_ready = false;
	serial_rx_state = SERIAL_RX_IDLE;
	serial_rx_cnt = 0;
	interrupts();
}

void serialSendPackage(short cmdid, byte *cmdbody, int cmdlen)
{
	byte buff[100];
	byte *pbuff = buff;
	int  cmd_len_all;
	
	cmd_len_all = 2 + 2 + 2 + cmdlen + 3;
	
	*pbuff++ = 0x55;
	*pbuff++ = 0xAA;

	*pbuff++ = (byte)((cmd_len_all-5) >> 8);
	*pbuff++ = (byte)(cmd_len_all-5);
	
	*pbuff++ = (byte)((cmdid | 0x8000) >> 8);
	*pbuff++ = (byte)cmdid;
	
	for ( int i = 0; i < cmdlen; i++ )
	{
		*pbuff++ = *cmdbody++;	
	}
	
	*pbuff++ = 0x00;
	*pbuff++ = 0x00;
	*pbuff++ = 0x0D;
	
	Serial.write(buff, cmd_len_all);

}
///////////////////////////////////////////////////////////////////////////////
void hsv_to_rgb (unsigned char h, unsigned char s, unsigned char v, CRGB * out)
{
	unsigned char r=0,g=0,b=0, i, f;
	unsigned int p, q, t;

	if( s == 0 ) {	
		r = g = b = v;
	}
	else
	{	i = h / 43;
		f = h % 43;
		p = (v * (255 - s))/256;
		q = (v * ((10710 - (s * f))/42))/256;
		t = (v * ((10710 - (s * (42 - f)))/42))/256;

		switch( i )
		{	
			case 0:
				r = v; g = t; b = p; break;
			case 1:
				r = q; g = v; b = p; break;
			case 2:
				r = p; g = v; b = t; break;
			case 3:
				r = p; g = q; b = v; break;			
			case 4:
				r = t; g = p; b = v; break;				
			case 5:
	 			r = v; g = p; b = q; break;
		}
	}
	out->r=r; out->g=g; out->b=b;
}

///////////////////////////////////////////////////////////////////////////////
int BH1750ADDR = 0x23;
byte bh_buff[2];
double BH1750_GetLux()
{
	 int i = 0;
	 double val = 0;

	 Wire.beginTransmission(BH1750ADDR);
	 Wire.write(0x10);
	 Wire.endTransmission();
	 delay(20);

	 Wire.beginTransmission(BH1750ADDR);
	 Wire.requestFrom(BH1750ADDR, 2);
	 while ( Wire.available() ) 
	 {
		bh_buff[i] = Wire.read();
		i++;
	 }
	 Wire.endTransmission();
	 if ( 2 == i )
	 {
		val=((bh_buff[0]<<8) | bh_buff[1])/1.2;
	 }
	 return val;
}

///////////////////////////////////////////////////////////////////////////////
// Main
CYaoji 	yaoji;

void setup()
{
	yaoji.init();
}

void loop()
{
	yaoji.updateSensors();					// Update all sensors
	yaoji.commWithClient();					// Communication with client
	yaoji.interactFromEnv();
	yaoji.decisionMake();
//	yaoji.takeAction();
	yaoji.refreshRGB();
//	yaoji.selfCheck();
}

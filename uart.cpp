#include "uart.h"

/*-------------------------------------------------------------------------
 * Constructor
 *-----------------------------------------------------------------------*/
template<class T>
uart<T>::uart(sc_module_name nm)
  : sc_module(nm)
{
  SC_HAS_PROCESS(uart);
  cout << "ConstructorCPP - UART " << name() <<endl;
  SC_THREAD(initUart);
  SC_THREAD(getGpsData_f);
  SC_THREAD(writeGpsData);
  SC_THREAD(sendGSMData);
}


/*-----------------------------------------------------------------------------
 * Methods/Threads
 *---------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------
 * Thread Name  : sendGSMData
 * Description  : Thread send the GSM data to Master CPU
 * Parameters   : void
 * Return Value : void
 *---------------------------------------------------------------------------*/
template<class T>
void uart<T>::sendGSMData(void)
{

  string gsmData = "GSM DATA";
  while(1)
  {
    /* Wait till character '!' is received */
    wait(send_gsm);
    /*----------------------------------------------------------------------
     * Write GSM data to Memory-01
     *--------------------------------------------------------------------*/
    cout << " " << endl;
    cout << BOLDWHITE << "Started writing GPS DATA to Memory-02............."
      << RESET << endl;
    for(unsigned int i = 0; i < gsmData.length(); i++)
    {
      uartTX02->write(gsmData[i]);
      cout << "GSM data - write : " << "  @ " << sc_time_stamp() 
          << ", data " << hex << gsmData[i] << endl;
      wait(1, SC_NS);
    }
    cout << BOLDWHITE << "Completed writing GPS DATA to Memory-02............."
      << RESET << endl;
  }
}

/*-----------------------------------------------------------------------------
 * Thread Name  : getGpsData_f
 * Description  : Thread reads GPS data from fifo and check for character '!'
 * Parameters   : void
 * Return Value : void
 *---------------------------------------------------------------------------*/
template<class T>
void uart<T>::getGpsData_f(void)
{
  char data = 0;
  unsigned int i = 0;
  while(1)
  {
    /* wait for data and read the data from fifo */
    wait(uartRX02->data_written_event());
    data = uartRX02->read();
    /* check character */
    if (data == '!')
    {
      /* notify to send GSM data to master CPU */
      send_gsm.notify(SC_ZERO_TIME);
      cout << GREEN << "Req     - Read : " << "  @ " << sc_time_stamp() 
          << ", data " << hex << data  << RESET << endl;
    }
    else
    {
      r_gps_data[i++] = data;
      cout << "GPS data - Read : " << "  @ " << sc_time_stamp() 
          << ", data " << hex << data << endl;
      cout << " " << endl;
      if (i == GPS_DATA_LEN)
      {
        cout << GREEN << "GPS Data received : " << r_gps_data << RESET << endl;
        i = 0;
        memset(r_gps_data, 0, GPS_DATA_LEN);
      }
    }
  }
}

/*-----------------------------------------------------------------------------
 * Thread Name  : writeGpsData
 * Description  : Thread Sends GPS data to Master CPU
 * Parameters   : void
 * Return Value : void
 *---------------------------------------------------------------------------*/
template<class T>
void uart<T>::writeGpsData(void)
{
  string uartData = "$GPGGA,181908.00,3404.7041778,N,07044.3966270,W,4,13,1.00,495.144,M,29.200,M,0.10,0000*40";

  while(1)
  {
    /*----------------------------------------------------------------------
     * Write GPS data to Memory-01
     *--------------------------------------------------------------------*/
    wait(2, SC_NS);
    cout << " " << endl;
    cout << BOLDWHITE << "Started writing GPS DATA to Memory-01............."
      << RESET << endl;

    for(unsigned int i = 0; i < uartData.length(); i++)
    {
      uartTX01->write(uartData[i]);
      cout << "GPS data - write : " << "  @ " << sc_time_stamp() 
          << ", data " << hex << uartData[i] << endl;
      wait(1, SC_NS);
    }
    cout << BOLDWHITE << "Completed writing GPS DATA to Memory-01............."
      << RESET << endl;
    cout << GREEN << "GPS Data " << uartData << RESET << endl;
    wait(10, SC_SEC);
  }
}

/*-----------------------------------------------------------------------------
 * Thread Name  : initUart
 * Description  : Main Thread which initate the tracking by sending character 'x'
 * Parameters   : void
 * Return Value : void
 *---------------------------------------------------------------------------*/
template<class T>
void uart<T>::initUart(void)
{
  char data = 0;

  wait(10, SC_SEC);
  /*----------------------------------------------------------------------
   * Read GPS data from slave-01
   *--------------------------------------------------------------------*/
  cout << " " << endl;
  for (int i = 0; i < NUM_LOC_REQ; i++)
  {
    cout << BOLDWHITE << "Requesting to read GPS DATA from slave1............. "
      << RESET << endl;
    data = 'x';
    /* Write Data to fifo */
    uartTX02->write(data);
    wait(100, SC_SEC);
  }

  cout << " " << endl;
  cout << "------------------------------------------------------" << endl;
  cout << "End of simulation " << endl;
  cout << "------------------------------------------------------" << endl;
  sc_stop();
}


//Medusa 3D-reconstruction


import 'dart:html';
//import 'package:three/three.dart';
import 'dart:async';


class Client
{
  ButtonElement button = querySelector('#continueButton');
  WebSocket ws;
  
  //Constructor
  Client()
  {
    
    connect();
    
    button.onClick.listen(sendToServer);
  }
  
  //Connect client to server
  void connect()
  {
    ws = new WebSocket('ws://127.0.0.1:7681'); 
    
    ws.onOpen.listen((e)  
    {
      onConnected();    
    });
  }
  
  void onConnected()
  {
    print('Connected');
    //ws.send('hej');
    
    ws.onMessage.listen((e) 
    {
      handleMessage(e.data);
    });
  }
  
  void handleMessage(data)
  { 
      print('något från servern');
  }
  
  
  void sendToServer(Event e) 
  {
    ws.send('hej');
  }
  
}

void main() 
{
  var client = new Client();
}


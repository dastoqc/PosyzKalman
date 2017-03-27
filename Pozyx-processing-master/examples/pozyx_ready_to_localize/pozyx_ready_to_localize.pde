import oscP5.*;
import processing.serial.*;
import java.lang.Math.*;

/////////////////////////////////////////////////////////////
//////////////////////  parameters //////////////////////////
/////////////////////////////////////////////////////////////

boolean serial = true;          // set to true to use Serial, false to use OSC messages.

int oscPort = 8888;               // change this to your UDP port
String serialPort = "COM9";      // change this to your COM port 
int gotanchors = 0;

int positionHistoryLength = 10;

// other parameters and variables

OscP5 oscP5;
Serial  myPort;

int     lf = 10;       //ASCII linefeed
String  inString;      //String for testing serial communication

// some variables for plotting the map
int offset_x = 30;
int offset_y = 30;
float pixel_per_mm = 0.5;
int border = 300;
int thick_mark = 300;
int device_size = 15;
int max[] = {10,10};
int min[] = {3000,3000};

// creates an empty list of pozyx devices
PozyxDevice[] pozyxDevices = {}; 


class PozyxDevice{
  private int ID;
  private int gridCell;
  private int[] pos_x = new int [positionHistoryLength];
  private int[] pos_y = new int [positionHistoryLength];
  private int[] pos_z = new int [positionHistoryLength];
  
  public PozyxDevice(int ID){this.ID = ID;}
  
  public void addPosition(int x, int y, int z, int g){
    System.arraycopy(pos_x, 0, pos_x, 1, positionHistoryLength - 1);
    System.arraycopy(pos_y, 0, pos_y, 1, positionHistoryLength - 1);
    System.arraycopy(pos_z, 0, pos_z, 1, positionHistoryLength - 1);
    
    pos_x[0] = x;
    pos_y[0] = y;
    pos_z[0] = z;
    
    gridCell = g;
  }
  
  public int[] getCurrentPosition(){
    int[] position ={pos_x[0], pos_y[0], pos_z[0]};
    return position;
  }
  public int getCurrentCell(){
    return gridCell;
  }
}

void setup(){
  size(1600, 2000, P3D);
  surface.setResizable(true);
  stroke(0, 0, 0);
  colorMode(RGB, 256); 
  // sets up the input
  if(serial){
    try{
      myPort = new Serial(this, serialPort, 115200);
      myPort.clear();
      myPort.bufferUntil(lf);
    }catch(Exception e){
      println("Cannot open serial port.");
    }
  }else{
    try{
      oscP5 = new OscP5(this, oscPort);
    }catch(Exception e){
      println("Cannot open UDP port");
    }
  }
}

void draw(){
  background(126,161,172);
  fill(0,0,0);
  text("(c) Pozyx Labs", width-100, 20);      
  drawMap();
}
  
void drawMap(){
  
  int plane_width =  width - 2 * offset_x;
  int plane_height = height - 2 * offset_y;
  // draw the plane
  stroke(0);
  fill(255);
  
  rect(offset_x, offset_y, plane_width, plane_height);
  
  calculateAspectRatio();
  
  pushMatrix();  
    
  translate(offset_x + (border * pixel_per_mm), height - offset_y - (border * pixel_per_mm));
  rotateX(radians(180));
  fill(0);
  
  // draw the grid
  strokeWeight(1);
  stroke(200);
  for(int i = 0; i <= (int) max[0]/thick_mark ; i++){
    /*print(min[0]*pixel_per_mm + i * thick_mark * pixel_per_mm);
    print(" ");
    print(min[1]*pixel_per_mm);
    print(" ");
    print(min[0]*pixel_per_mm + i * thick_mark * pixel_per_mm);
    print(" ");
    println(min[1]*pixel_per_mm + plane_height);*/
    line(min[0]*pixel_per_mm + i * thick_mark * pixel_per_mm, min[1]*pixel_per_mm, min[0]*pixel_per_mm + i * thick_mark * pixel_per_mm, max[1]*pixel_per_mm );
  }
    
  stroke(100);
  for(int i = 0; i <= (int) max[1]/thick_mark ; i++)
    line(min[0]*pixel_per_mm, min[1]*pixel_per_mm + i * thick_mark * pixel_per_mm, max[0]*pixel_per_mm ,  min[1]*pixel_per_mm + (i* thick_mark * pixel_per_mm));
  
  drawDevices();
    
  stroke(0);
  fill(0);
  drawArrow(0, 0, 50, 0.);
  drawArrow(0, 0, 50, 90.);
  pushMatrix();
  rotateX(radians(180));
  text("X", 55, 5);  
  text("Y", -3, -55);
  popMatrix();
  
  popMatrix();  
}

void calculateAspectRatio(){
  float plane_width =  width - 2 * offset_x;
  float plane_height = height - 2 * offset_y;
  int max_width_mm = 0;
  int max_height_mm = 0;
  for (PozyxDevice pozyxDevice : pozyxDevices){
    int[] pos = pozyxDevice.getCurrentPosition();
    max_width_mm = max(max_width_mm, pos[0]);
    max_height_mm = max(max_height_mm, pos[1]);
  }
  max_width_mm += 2*border; 
  max_height_mm += 2*border; 
  pixel_per_mm = min(pixel_per_mm, plane_width / max_width_mm, plane_height / max_height_mm);
}

void drawDevices(){
  for(PozyxDevice pozyxDevice : pozyxDevices){
    drawDevice(pozyxDevice);
  }
}

void drawDevice(PozyxDevice device){  
  stroke(0, 0, 0);
  fill(0, 0, 0);
  int[] current_position = device.getCurrentPosition();
  int gcell = device.getCurrentCell();
  ellipse(pixel_per_mm * current_position[0] - device_size/2, pixel_per_mm * current_position[1] + device_size/2, device_size, device_size);
  
  pushMatrix();
  rotateX(radians(180));
  if(gcell!=0 && gotanchors==4) {
    stroke(0);
    fill(200);
    int c_per_r = (max[0]-min[0])/thick_mark;
    int r = 1;
    if (c_per_r < gcell) {
      r = (int)(gcell/c_per_r) + 1;
    }
    int c = gcell - (r-1)*c_per_r;
    //print(hex(device.ID, 4));print(" ");print(pixel_per_mm * current_position[0] - 3 * device_size / 2);print(" ");println(- pixel_per_mm * current_position[1] + device_size);
    //print(gcell);print(" ");print(c);print(" ");print(c_per_r);print(" ");println(r);
    rect((min[0]+(c-1)*thick_mark)*pixel_per_mm,  -(min[1]+(r)*thick_mark)*pixel_per_mm,  (thick_mark)*pixel_per_mm, (thick_mark)*pixel_per_mm);
  }
  
  fill(0);
  textSize(11);
  text("0x" + hex(device.ID, 4), pixel_per_mm * current_position[0] - 3 * device_size / 2, - pixel_per_mm * current_position[1] + device_size);
  textSize(12);
  
  
  
  popMatrix();
}

void addPosition(int ID, int x, int y, int z, int g){
  for(PozyxDevice pozyxDevice : pozyxDevices){
    // ID in device list already
    if(pozyxDevice.ID == ID){
      pozyxDevice.addPosition(x, y, z, g);
      return;
    }
  }
  // ID not in device list
  PozyxDevice newPozyx = new PozyxDevice(ID);
  newPozyx.addPosition(x, y, z, g);
  pozyxDevices = (PozyxDevice[]) append(pozyxDevices, newPozyx);
}

void drawArrow(int center_x, int center_y, int len, float angle){
  pushMatrix();
  translate(center_x, center_y);
  rotate(radians(angle));
  strokeWeight(2);
  line(0,0,len, 0);
  line(len, 0, len - 8, -8);
  line(len, 0, len - 8, 8);
  popMatrix();
}

void serialEvent(Serial p) {
  // expected string: POS,network_id,posx,posy,posz
  inString = (myPort.readString());
  print(inString);
  try {
    // parse the data
    String[] dataStrings = split(inString, ',');
    
    if (dataStrings[0].equals("POS")){
      int id = Integer.parseInt(split(dataStrings[1], 'x')[1], 16);
      addPosition(id, int(dataStrings[2]), int(dataStrings[3]), int(dataStrings[4]), int(dataStrings[5]));
      //print(id);print(" ");print(int(dataStrings[2]));print(" ");print(int(dataStrings[3]));print(" ");print(int(dataStrings[4]));print(" ");println(int(dataStrings[5]));
    }
    if (dataStrings[0].equals("ANCHOR")){
      gotanchors++;
      int id = Integer.parseInt(split(dataStrings[1], 'x')[1], 16);
      if(int(dataStrings[2])<min[0])
        min[0]=int(dataStrings[2]);
      else if(int(dataStrings[2])>max[0])
        max[0]=int(dataStrings[2]);
      if(int(dataStrings[3])<min[1])
        min[1]=int(dataStrings[3]);
      else if (int(dataStrings[3])>max[1])
        max[1]=int(dataStrings[3]);
      addPosition(id, int(dataStrings[2]), int(dataStrings[3]), int(dataStrings[4]), 0);
    }
  }catch (Exception e) {
      println("Error while reading serial data.");
  }
}

void oscEvent(OscMessage theOscMessage) {
  // osc message received
  if (theOscMessage.addrPattern().equals("/position") || theOscMessage.addrPattern().equals("/anchor")){
    try{
      addPosition(theOscMessage.get(0).intValue(), theOscMessage.get(1).intValue(), theOscMessage.get(2).intValue(), 0, 0);
    }catch(Exception e){
      println("Error while receiving OSC position");
    }
  }
}

void keyPressed() {
  // resets the scale to the current area taken by the devices. Useful if you moved too much outside of the anchor area.
  pixel_per_mm = 0.5;
  println("Resetting area size");  
}
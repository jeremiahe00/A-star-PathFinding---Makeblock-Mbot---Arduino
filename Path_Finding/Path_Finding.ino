#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeMCore.h>
//#include <QueueList_Modified.h>
#define row 10
#define col 10

byte goalN; // goal position on grid
//byte path[75]; byte pN=0 
byte openList[100]; // contains all the possible paths
byte closedList[100]; // contains the path taken
byte oLN=0, cLN=0, bM=0; // the counters for the openList, closedList and the bot's movement
bool gFound = false, openLEmpty = false, isMoved = false;
byte curBotPos; // holds current bot position
char botM[75]; // contains the specific moves the bot should take to reach its goal

struct Node
{
  byte g, h, f;
  byte parent;
  byte index;
  byte gridNom;
};


struct Grid
{
  Node Map[row][col];
} PF ;


byte H(byte curR, byte curC, byte goalS)  // manhattan distance heauristics function
{
 byte rowg, colg;
 byte manhattan=0;

 
   rowg = (byte)goalS/10;
   colg = goalS%10;
   manhattan += (abs(curR - rowg) + abs(curC - colg));
   
  return manhattan;
}


byte G(byte curR, byte curC)  // returns the "depth" level of the tile
{
  byte gValue, parInd;
  byte rowg, colg;
  parInd = PF.Map[curR][curC].parent;
 
  rowg = (byte)parInd/10;
  colg = parInd%10;
  gValue = PF.Map[rowg][colg].g;
  
  return (gValue+1);
}

byte FV(byte curG, byte curH) // the "cost" of the path taken; adds H and G values for each tile
{
 byte fValue; 
  
  fValue = curG + curH;
  return fValue;
}




MeDCMotor motor_9(9); // initalizes the left motor
MeDCMotor motor_10(10); // initializes the right motor
void move(byte direction, byte speed) // sets up potential movements for the robot; *needed*
{
      byte leftSpeed = 0;
      byte rightSpeed = 0;
      if(direction == 1){
          leftSpeed = speed;
          rightSpeed = speed;
      }else if(direction == 2){
          leftSpeed = -speed;
          rightSpeed = -speed;
      }else if(direction == 3){
          leftSpeed = -speed;
          rightSpeed = speed;
      }else if(direction == 4){
          leftSpeed = speed;
          rightSpeed = -speed;
      }
      motor_9.run((9)==M1?-(leftSpeed):(leftSpeed));
      motor_10.run((10)==M1?-(rightSpeed):(rightSpeed));
}
double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;
//double currentTime = 0;
//double lastTime = 0;
//double getLastTime(){
      //return currentTime = millis()/1000.0 - lastTime; }

void setup(){ // sets up the program, builds the map, prints the grid for representation purposes, and takes user input for the goal
  Serial.begin(9600);
  buildMap();
  printGrid1();
  printGrid2();
  setGoal();

}

void loop(){ // the "main" program
  if (!isGoal(curBotPos) && OLE)
  {
    _loop(); // the actual performance of the A* algorithm
  }
  else if (isGoal(curBotPos) && !isMoved)
  {
    Serial.println("Goal Reached");
    delay(10000);
    movement();
    move(1, 0);
    motor_9.run((9)==M1?-(0):(0));
    motor_10.run((10)==M1?-(0):(0));
    
    for (byte j = 0; j < bM; j++)
    {
      Serial.print(botM[j]);
      Serial.print(" ");
    }
  Serial.println();
  }
    
}

void _delay(float seconds){
    long endTime = millis() + seconds * 1000;
    while(millis() < endTime)_loop();
}

void _loop(){ // performs the A* algorithm
  possMov(curBotPos);
  for (byte i = 0; i < oLN; i++)
  {
    Serial.println(openList[i]);
  }
  Serial.println();
  AddClosedList();
  
  for (byte i = 0; i < oLN; i++)
  {
    Serial.println(openList[i]);
  }
  Serial.println();
  for (byte j = 0; j < cLN; j++)
  {
    Serial.println(closedList[j]);
  }
  Serial.println();
  printGrid2();
  
}

void buildMap() // builds the 10x10 map grid
{
  byte gridIn = 0;
  for (byte i = 0; i < row; i++)
  {
   for (byte j = 0; j < col; j++)
   {
    PF.Map[i][j].gridNom = gridIn;
    PF.Map[i][j].index = 0;
    PF.Map[i][j].parent = 0;
    PF.Map[i][j].h = 0;
    PF.Map[i][j].g = 0;
    PF.Map[i][j].f = 0;
    
    gridIn++;    
   }
  }
}

void printGrid1()  // prints the grid, using indices 0 to 99 to represent the possible paths
{
  for (byte i = 0; i < row; i++)
  {
   for (byte j = 0; j < col; j++)
   {
    Serial.print(PF.Map[i][j].gridNom);
    if (j != 9)
    {
      if (PF.Map[i][j].gridNom < 9)
      {
        Serial.print("  | ");
      }
      else
    Serial.print(" | ");
    } 
   }
  
  if (i != 9)
    {
      Serial.println();
    Serial.print("-------------------------------------------------");
    Serial.println();
    }
  }
  Serial.println();
  Serial.println();
}

void printGrid2() // prints the grid, 0 - untravelled | 1 - travelled | 2 - obstacles | 3 - goal
{
  for (byte i = 0; i < row; i++)
  {
   for (byte j = 0; j < col; j++)
   {
    Serial.print(PF.Map[i][j].index);
    if (j != 9)
    {
    Serial.print(" | ");
    } 
   }
  
  if (i != 9)
    {
      Serial.println();
    Serial.print("--------------------------------------");
    Serial.println();
    }
  }
  Serial.println();
  Serial.println();
}

void setGoal() // asks user for input to set the goal state/tile
{
  byte goal;
  Serial.println("Where do you want to place your goal state?");
  Serial.println("Using the numbers displayaed in the earlier grid, enter a number to intialize as your goal state.");
  Serial.println();

  while (!Serial.available() )
  {
     goal = Serial.parseInt();
  }
 
  for (byte i = 0; i < row; i++)
  {
    for (byte k = 0; k < col; k++)
    {
      if (PF.Map[i][k].gridNom == goal)
      {
        
        PF.Map[i][k].index = 3;
        goalN = PF.Map[i][k].gridNom;
      }
      if (PF.Map[i][k].gridNom == 45)
      {
        PF.Map[i][k].index = 1;
        curBotPos = PF.Map[i][k].gridNom;
      }
      else if (PF.Map[i][k].gridNom == 26 || PF.Map[i][k].gridNom == 32 || PF.Map[i][k].gridNom == 64 || PF.Map[i][k].gridNom == 71 || PF.Map[i][k].gridNom == 77)
      {
        PF.Map[i][k].index = 2;
      }
      else
      PF.Map[i][k].index = 0;
    }
  }
  printGrid2();
}

void possMov(byte gridNom) // checks the possible moves depending on the location of the current tile the bot is on
{
  byte rowp = (byte) gridNom / 10;
  byte colp = gridNom % 10;
  if (gridNom == 0) // checks the corner tiles | 2 possible moves
  {
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }

    if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 10);
    }
  }
  else if (gridNom == 9)
  {
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }

    if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 10);
    }
  }
  else if (gridNom == 90)
  {
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }

    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 10);
    }
  }
  else if (gridNom == 99)
  {
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 10);
    }

    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 1);
    }   
  }
  else if (gridNom > 0 && gridNom < 9) // checks the tiles on the outermost edges of the map | 3 possible moves
  {
   if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    } 
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }
       if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 10);
    }    
  }
  else if (gridNom%10==0)
  {
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 10);
    } 
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }
       if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 10);
    }
  }
  else if (gridNom%10==9)
  {
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 10);
    } 
    if (PF.Map[rowp][colp- 1].index != 1 && PF.Map[rowp][colp- 1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }
       if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 10);
    }
  }
  else if (gridNom > 90 && gridNom < 99)
  {
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 10);
    } 
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }
       if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }
  }
  else { // checks the remaining tiles | 4 possible moves
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 10);
    } 
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }
       if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
  }
     if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 10);
  }
}

}

void AddOpenList(byte aol) // adds the potential possible moves to the openList
{
  
  openList[oLN++] = aol;
  heuristics(aol);
}

void heuristics(byte curIn) // calculates the "cost" of the tile
{
  byte hH, gH, fH;
  byte rowh = (byte) curIn / 10;
  byte colh = curIn % 10;

  hH = H(rowh, colh, goalN);
  PF.Map[rowh][colh].h = hH;
  gH = G(rowh, colh);
  PF.Map[rowh][colh].g = gH;
  fH = FV(hH,gH);
  PF.Map[rowh][colh].f = fH;
}

byte getNextFI() // returns the best heuristics value restricted by the current path the bot is taking
{
  byte rowf;
  byte colf;
  byte lowestF;
  byte lowest = openList[0];
  rowf = (byte) lowest / 10;
  colf = lowest % 10;
  lowestF = PF.Map[rowf][colf].f;
  
  for (byte i = 0; i < oLN; i++)
  {
    rowf = (byte) openList[i] / 10;
    colf = openList[i] % 10;
    
    if (PF.Map[rowf][colf].f <= lowestF) 
    {
      lowestF = PF.Map[rowf][colf].f;
      lowest = rowf*10 + colf;
    }
  }
  
  return lowest;
}

void AddClosedList() // adds the "best" tile to the closedList
{
  byte low = getNextFI(); 
  byte rowa, cola;

  closedList[cLN++] = low;
  //path[pN] = low;
  rowa = (byte)low/10;
  cola = low%10;
  PF.Map[rowa][cola].index = 1;
  curBotPos = low;
  removeFOL(low);
  
  
}

void removeFOL(byte rfol) // removes previous potential paths from the openList, in order to get the "best" current path
{
  byte rm = 0;
  
  //while (openList[rm] != rfol)
  //{
  //  rm++;
  //}
  
  for (byte i = rm; i < oLN-3; i++)
  {
    if (openList[i] == rfol)
    {
      openList[i] = openList[i+1];
    }
    else
      openList[i] = openList[i+2];
  }
    oLN=oLN-2;
}

bool OLE() // checks if the openList is empty
{
  if (oLN == 0)
  {
    return true;
  }
  else
  return false;
}

bool isGoal(byte ig) // checks if the goal has been reached
{
  if (ig == goalN)
  {
    return true; 
  }
  else
  return false;
}

bool alreadyOnOL(byte rowaol, byte colaol) // checks if the tile is already on the openList
{
  byte indexol;
  bool on = false;

  indexol = rowaol*10 + colaol;
  for (byte i = 0; i < oLN; i++)
  {
    if (openList[i] == indexol)
    {
      on = true;
    }
  }
  
  return on;
}

void movement() // performs the actual robots movement according to the path chosen by the algorithm
{ // turns the robot accoridingly, and changes its orientation to face "front" after every move
  byte rowm, colm, parm;
  
  for(byte index = 0; index < cLN; index++)
  {
    rowm = (byte)closedList[index]/10;
    colm = closedList[index]%10;
    parm = PF.Map[rowm][colm].parent;

    if (closedList[index] == parm - 10)
    {
      botM[bM++] = 'u';
      delay(500);
      forward();
      delay(500);
    }
    else if (closedList[index] == parm - 1)
    {
      botM[bM++] = 'l';
      delay(500);
      leftturn();
      delay(500);
      forward(); 
      delay(500);
      rightturn();
      delay(500);
    }
    else if (closedList[index] == parm + 1)
    {
      botM[bM++] = 'r';
      rightturn();
      delay(500);
      forward();
      delay(500);
      leftturn();
      delay(500);
      
    }
    else if (closedList[index] == parm + 10)
    {
      botM[bM++] = 'd';
      delay(500);
      backwards();
      delay(500);
      forward();
      delay(500);
      backwards();
      delay(500);
    }
  }
  
  isMoved = true;
}

void forward() // moves the robot forward
{
  unsigned long time = millis();
  while (millis() < time+1500)
  {
    move(1, 200);
    Serial.println('f');
    if (millis() > time+1500)
    {
      move(1, 0);
    }
  }
}

void leftturn() // turns the robot left
{
  unsigned long time = millis();
  while (millis() < time+1000)
  {
    //move(4, 300);
    motor_10.run((10)==M1?-(130):(130));
    Serial.println('l');
    if (millis() > time+1000)
    {
      //move(1, 0);
      motor_10.run((10)==M1?-(0):(0));
    }
  }
  //forward();
}

void rightturn() // turns the robot right
{
  unsigned long time = millis();
  while (millis() < time+1000)
  {
    //move(4, 300);
    motor_9.run((9)==M1?-(130):(130));
    Serial.println('r');
    if (millis() > time+1000)
    {
      //move(1, 0);
      motor_9.run((9)==M1?-(0):(0));
    }
  }
  //forward(); 
}

void backwards() // turns the robot around
{
  rightturn();
  rightturn();
}

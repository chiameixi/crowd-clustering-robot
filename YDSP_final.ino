#include "SoftwareSerial.h"
SoftwareSerial mySerial(2, 3); // RX | TX

#include<Servo.h>
Servo servoB;
Servo servoD;
Servo ultrasonicServo;

//servoD: stops at 0 and moves counterclockwise at 90 & 180
//servoB: moves clockwise at 0, stops at 90 and moves counterclockwise at 180

#define soundspeed 0.0343
#define trigpin A1 //any analog pins work, with exception of A4, A5
#define echopin A0

//servos
#define svB 10
#define svD 11
#define us 5

//led
#define red 8
#define amber 7
#define green 6

int const num_of_device = 4;
int timeout = 48; // About 60 seconds
String string_b = "";
int const num_of_sets_of_distances = 4;
float const e = 2.71828;
int threshold = 0; //threshold is the max number of people in a cluster that can gather -- change to 5
int distance_robot = 10; //the distance the robot moves is 100cm (1m) -- change to 100

// Function Prototypes (mega functions)
void collectBluetoothData(int set_number);
void computeCluster();
void computeDirection();
void moveForward();
void moveBackward();
void rotate(); //the robot rotates right on the spot
void stopMovement();

//Function Prototypes: mini-functions under collectBluetoothData
String inquire_BT_devices(int num_of_device);
float update_RSSI_distance(int set_number, int l, String rssi);
float append_RSSI_distance(int set_number, int l, String rssi);
//void deviceType_conversion_hexaTobinary(int size_device, String deviceType, String deviceTypelist[50]);

//Function Prototypes: mini-functions under computeCluster
void sortDiffinDist();
void sortdiffbetweenset1and2(int x, int y);

void setup()
{
  //setup for movement functions
  servoB.attach(svB);
  servoD.attach(svD);
  ultrasonicServo.attach(us);
  //pin must be pwm since it is used to control servo Attach Servo Pin to sg90

  Serial.begin(9600);
  mySerial.begin(38400);
  Serial.println("***AT commands mode***");

  mySerial.write("AT+RESET\r\n");
  delay(1000);
  while (mySerial.available())
    Serial.write(mySerial.read());

  mySerial.write("AT+ORGL\r\n");
  delay(1000);
  while (mySerial.available())
    Serial.write(mySerial.read());

  mySerial.write("AT+ROLE=1\r\n");
  delay(1000);
  while (mySerial.available())
    Serial.write(mySerial.read());

  mySerial.write("AT+RESET\r\n");
  delay(1000);
  while (mySerial.available())
    Serial.write(mySerial.read());

  mySerial.write("AT+INIT\r\n");
  delay(1000);
  while (mySerial.available())
    Serial.write(mySerial.read());

  string_b = "AT+INQM=1," + String(num_of_device) + "," + String(timeout) + "\r\n";
  mySerial.write(&string_b[0]);
  delay(1000);
  while (mySerial.available())
    Serial.write(mySerial.read());

  pinMode(trigpin, OUTPUT);
  pinMode(echopin, INPUT);

  //stopMovement();
  //delay(1000);
}

float RSSIdistance;
//String personData3[5][2]; //array of distances & bluetooth addresses
int size_device;
String list_of_bluetooth_addresses[num_of_device]; //max number of devices ever collected in a set
int rssi_distance_values[num_of_sets_of_distances][num_of_device]; //adding 4 sets of distance data
int valid_index[num_of_device];
int invalid_index[num_of_device];

int count = 0; //tells us the unique number of bluetooth addresses for the first set but they might or might not be valid!
int count2 = 0; //number of invalid indexes
int count3 = 0; //number of valid indexes = number of valid unique bluetooth addresses

//for angle determination
int lengthOfclusters[num_of_device];
int index_illegalClusters[num_of_device];
int anglesOfclusters[num_of_device];

//for detecting obstacles
long timeTaken = 0;
long distance = 0;

//for moving of the robot to the cluster & rotation of the robot -- can be changed accordingly
int delay_time_100cm = 3800;
int delay_time_60degrees = 300;
for (int x = 0; x < num_of_device; x++) {
  if (rssi_distance_values[0][x] != 0) {
    count += 1; //as long as there is a value in the 1st set of rssi distances, it is a unique bluetooth address so increment count
  }
}
//adding invalid indexes into the list of invalid indexes
for (int x = 0; x < num_of_device; x++) { //use count + 1 just in case you have a repeated bluetooth address in the middle
  if (rssi_distance_values[0][x] == 0 || rssi_distance_values[1][x] == 0 || rssi_distance_values[2][x] == 0 || rssi_distance_values[3][x] == 0) {
    invalid_index[count2] = x;
    count2 += 1;
  }
  else {
    valid_index[count3] = x;
    count3 += 1;
  }
}

digitalWrite(green, LOW);

float sum_of_distances = 0;
computeCluster();
computeDirection();

//after everything is done
digitalWrite(green, HIGH);
digitalWrite(amber, LOW);
digitalWrite(red, LOW);

Serial.println("NEXT void loop");
while (1) {};
}

//---------------------------------------MOVEMENT FUNCTIONS---------------------------------------
void moveForward() {
  servoD.write(90);
  servoB.write(0);
}

void moveBackward() { //turn right by 180 degrees and moveForward
  rotate();
  delay(885); //turn right by 180 degrees
  moveForward();
}

void rotate() {
  servoD.write(90);
  servoB.write(180);
}

void stopMovement() {
  servoD.write(0);
  servoB.write(90);
}

void rotate60andMove() {
  stopMovement();
  delay(1000);
  rotate();
  delay(325);//265
  stopMovement();
  delay(1000);

  moveForward();
  delay(3000);  //1145 ~30cm, 3000 was just for testing on larger scale
  stopMovement();
  delay(1000);
}

void rotate120andMove() {
  stopMovement();
  delay(1000);
  rotate();
  delay(655); //700
  stopMovement();
  delay(1000);

  moveForward();
  delay(3000);
  stopMovement();
  delay(1000);
}

//----------------------------------DETECTING OBSTACLES----------------------------------
void getSonarDistance() {
  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);
  timeTaken = pulseIn(echopin, HIGH);
  distance = soundspeed * timeTaken / 2;
  Serial.println(distance);
  delay(15);
}

void detectObstacle() {
  for (int angle = 0; angle < 180; angle++) {
    ultrasonicServo.write(angle);
    delay(15);
    getSonarDistance();
    if (distance > 0 && distance < 20 ) { //unit is in cm
      Serial.println("near object");
      //stopMovement();
      digitalWrite(red, HIGH);
      //delay(1000); //can remove later?
      rotate();
      delay(1000);

      moveBackward();
      delay(500);
      //stopMovement();
      rotate();
      delay(435);
      //450 for 90 degrees, minus 15 in built in moveright()
      moveForward();
      delay(500);
      digitalWrite(red, LOW);

    }
    timeTaken = 0;
    distance = 0;
  }
  if (distance >= 20) {
    moveForward();
  }

  // Sweep servo from 180 to 0 degrees
  for (int angle = 180; angle > 0; angle--) {
    ultrasonicServo.write(angle);
    delay(15);
    getSonarDistance();
    if (distance > 0 && distance < 20 ) { //unit is in cm
      Serial.println("near object");
      //stopMovement();
      digitalWrite(red, HIGH);
      moveBackward();
      delay(500);
      //stopMovement();
      rotate();
      delay(435);
      //450 for 90 degrees, minus 15 in built in moveright()
      moveForward();
      delay(500);
      digitalWrite(red, LOW);

      timeTaken = 0;
      distance = 0;
    }

    if (distance >= 20) {
      moveForward();
    }
  }
}

//----------------------------------COLLECT BLUETOOTH DATA----------------------------------
void collectBluetoothData(int set_number)
{
  //inquires bluetooth device
  String string_b = inquire_BT_devices(num_of_device);
  //String string_b = "+INQ:18CE:94:FB5DBD,5A020C,FFD9+INQ:18CE:97:FB5DBD,5A020C,FFD8+INQ:15HJ:35:BO2SH3,5A020C,FFD6";
  Serial.println(string_b);

  //since b is actually going to be a continuous flow of people's bluetooth address, device type & rssi value, we need to separate them
  int NUM_OF_HUMAN = 0; //this will be the number of devices collected which might not correspond to the max no. of devices (since it might not detect that many, depends on the max no.)
  for (int i = 0; i < string_b.length(); i++) {
    if (string_b[i] == '+') {
      NUM_OF_HUMAN += 1;
    }
  }

  int indexList[NUM_OF_HUMAN + 1]; //adds the indexes of where +INQ: is present so that it can be removed and the two data samples can be separated

  for (int y = 0; y < NUM_OF_HUMAN + 1; y++) {
    if (string_b.indexOf("+INQ:") != -1) { //while +INQ: is present in the string
      int index1 = string_b.indexOf("+INQ:");
      indexList[y] = index1;
      string_b.remove(index1, 5);
    }
    else {
      indexList[y] = -1;
      break;
    }
  }

  String personData; //contains the substrings
  String personData2[num_of_device]; //contains an array of the substrings separated (the two data samples will be separated)

  for (int y = 0; y < NUM_OF_HUMAN; y++) {
    personData = "";
    personData = string_b.substring(indexList[y], indexList[y + 1]); //will be "2:72:D2224,3E0104,FFBC"
    personData2[y] = personData;
  }

  //so at this stage, personData2 is [":2:72:D2224,3E0104,FFBC", "2:45:D2374,3E0904,FFBE"] so the next step is to make it into something like this: [[":2:72:D2224", "3E0104", "FFBC"], ["2:45:D2374", "3E0904", "FFBE"]]

  int indexList2[num_of_device][4]; //each data would have 2 commas and the start and end indexes that need to be added

  for (int y = 0; y < NUM_OF_HUMAN; y++) {
    for (int i = 0; i < 4; i++) {
      if (i == 0) {
        indexList2[y][i] = 0;
      }
      else if (i == 1) {
        if (personData2[y].indexOf(",") != -1) { //if comma is present in the string
          int index2 = personData2[y].indexOf(",");
          indexList2[y][i] = index2;
          personData2[y].remove(index2, 1); //to remove the comma so that it can detect the next comma
        }
        else {
          break;
        }
      }
      else if (i == 2) {
        if (personData2[y].indexOf(",") != -1) { //if comma is present in the string
          int index2 = personData2[y].indexOf(",");
          indexList2[y][i] = index2;
          personData2[y].remove(index2, 1);
        }
      }
      else if (i == 3) {
        indexList2[y][i] = -1;
      }
    }
  }

  for (int y = 0; y < NUM_OF_HUMAN; y++) {
    //String deviceTypelist[50]; //to add the binary deviceTypes
    String bluetoothAddress = personData2[y].substring(indexList2[y][0], indexList2[y][1]); //bluetooth address
    //String deviceType = personData2[y].substring(indexList2[y][1], indexList2[y][2]); //device type conversion from hexadecimal to binary
    //int size_device = deviceType.length();
    //deviceType_conversion_hexaTobinary(size_device, deviceType, deviceTypelist);
    //if (deviceTypelist[size_device - 3] == "0010" && deviceTypelist[size_device - 4][3] == '0') { //filtering for mobile phones
    for (int l = 0; l < NUM_OF_HUMAN; l++) { //searching for repeated bluetoothaddresses and not adding the repeated ones within the same set
      if (set_number == 0) {
        if (list_of_bluetooth_addresses[l] == bluetoothAddress) {
          //updating the RSSI distance (if the same bluetooth address is collected more than once, we update the RSSI distance by taking the mean of the distances)
          String rssi = personData2[y].substring(indexList2[y][2], indexList2[y][3]);
          float updated_RSSIdistance = update_RSSI_distance(set_number, l, rssi); //to ensure that the same bluetooth address is not collected again and its corresponding RSSI distance is not added, but instead the mean distance is taken & added earlier on
          if (l != y) { //just in case something wrong happens
            list_of_bluetooth_addresses[y] = ""; //we are only going to add the bluetooth addresses for the first set and filter for the rest!
            rssi_distance_values[set_number][y] = 0; //if there is any rssi value added into the y index, we must delete that, same for the bluetooth address
          }
          break;
        }
        else if (list_of_bluetooth_addresses[l] != bluetoothAddress) { //only appending unique bluetooth addresses and the corresponding RSSI distance value
          list_of_bluetooth_addresses[y] = bluetoothAddress; //bluetooth address
          String rssi = personData2[y].substring(indexList2[y][2], indexList2[y][3]);
          float RSSIdistance = append_RSSI_distance(set_number, y, rssi);
          continue;
        }
      }

      else if (set_number != 0) {
        if (list_of_bluetooth_addresses[l] == bluetoothAddress) { //bluetooth address must be initially collected as well (MUST BE present in the first set)
          if (rssi_distance_values[set_number][l] == 0) { //this means that no other bluetooth data collected belonged to this address and you can just simply add the rssi data in
            String rssi = personData2[y].substring(indexList2[y][2], indexList2[y][3]);
            float RSSIdistance = append_RSSI_distance(set_number, l, rssi);
            //Serial.println(RSSIdistance);
          }
          else if (rssi_distance_values[set_number][l] != 0) { //this means that this bluetooth address has been collected earlier so we need to take the mean of the rssi distances
            String rssi = personData2[y].substring(indexList2[y][2], indexList2[y][3]);
            float updated_RSSIdistance = update_RSSI_distance(set_number, l, rssi);
            //Serial.println(updated_RSSIdistance);
          }
        }
      }
      //}
    }
  }
  //Serial.println("NEXT SET");
  delay(2000);
}

//FUNCTIONS FOR collectBluetoothData()
String inquire_BT_devices(int num_of_device)
{
  String string_b = "";
  mySerial.write("AT+INQ\r\n");

  for (int i = 0; i < num_of_device * 10; i++) // Delay loop, worst-case time = (num_of_device * 1 second)
  {
    if (!mySerial.available())
      delay(100); // Allows buffering, 100ms to prevent buffer overflow
    while (mySerial.available())
      string_b.concat(char(mySerial.read())); // Read buffer while data available
  }

  return string_b;
}

float update_RSSI_distance(int set_number, int l, String rssi) { //for those that have repeated bluetooth addresses
  float outdated_RSSIdistance;
  outdated_RSSIdistance = float(rssi_distance_values[set_number][l]);
  char rssiChar[5];
  rssi.toCharArray(rssiChar, 5);
  int rssiConverted = (int)strtol( rssiChar, NULL, 16 );

  float rssi_float;
  rssi_float = float(rssiConverted);
  float incoming_RSSIdistance;
  incoming_RSSIdistance = 0.0237 * pow(e, (-0.0661 * rssi_float)) * 100; //the distance will be in cm
  rssi_distance_values[set_number][l] = (outdated_RSSIdistance + incoming_RSSIdistance) / 2;

  return rssi_distance_values[set_number][l];
}

float append_RSSI_distance(int set_number, int l, String rssi) { //for those that have unique bluetooth address
  char rssiChar[5];
  rssi.toCharArray(rssiChar, 5);
  int rssiConverted = (int)strtol( rssiChar, NULL, 16 );

  float rssi_float;
  rssi_float = float(rssiConverted);
  float RSSIdistance;
  RSSIdistance = 0.0237 * pow(e, (-0.0661 * rssi_float)) * 100; //the distance will be in cm
  rssi_distance_values[set_number][l] = RSSIdistance;

  return RSSIdistance;
}

/*void deviceType_conversion_hexaTobinary(int size_device, String deviceType, String deviceTypelist[50]) {
  for (int z = 0; z < size_device; z++) {
    if (deviceType[z] == '0') {
      deviceTypelist[z] = "0000";
    } else if (deviceType[z] == '1') {
      deviceTypelist[z] = "0001";
    } else if (deviceType[z] == '2') {
      deviceTypelist[z] = "0010";
    } else if (deviceType[z] == '3') {
      deviceTypelist[z] = "0011";
    } else if (deviceType[z] == '4') {
      deviceTypelist[z] = "0100";
    } else if (deviceType[z] == '5') {
      deviceTypelist[z] = "0101";
    } else if (deviceType[z] == '6') {
      deviceTypelist[z] = "0110";
    } else if (deviceType[z] == '7') {
      deviceTypelist[z] = "0111";
    } else if (deviceType[z] == '8') {
      deviceTypelist[z] = "1000";
    } else if (deviceType[z] == '9') {
      deviceTypelist[z] = "1001";
    } else if (deviceType[z] == 'A') {
      deviceTypelist[z] = "1010";
    } else if (deviceType[z] == 'B') {
      deviceTypelist[z] = "1011";
    } else if (deviceType[z] == 'C') {
      deviceTypelist[z] = "1100";
    } else if (deviceType[z] == 'D') {
      deviceTypelist[z] = "1101";
    } else if (deviceType[z] == 'E') {
      deviceTypelist[z] = "1110";
    } else if (deviceType[z] == 'F') {
      deviceTypelist[z] = "1111";
    }
  }
  }*/

//----------------------------------COMPUTE CLUSTER----------------------------------
int r;
int c;
int final_counter;
int counter5 = 0;
String allclusters[num_of_device][num_of_device]; //the max of the number of clusters found would be NUM_OF_HUMAN since the max number of people being examined at any one time is 50
int counter2;
int num_of_clustered_people;
int total_num_of_people_clustered = 0;
int counter6;
int counter7;

struct diff_dist_struct {
  String bluetooth_address;
  float distance_value;
};

struct people {
  String bluetooth_address;
  float distance_value;
};

people arrayofdistances[3][num_of_device]; //to store all the data

diff_dist_struct diffInDistSet1and2[num_of_device];
//can loop through structures to see bluetooth content
diff_dist_struct diffInDistSet2and3[num_of_device];

void computeCluster() {
  for (int x = 0; x < 3; x++) {
    //transfers all the data to arrayofdistances for further manipulation!
    for (int z = 0; z < count3; z++) {
      arrayofdistances[x][z].bluetooth_address = list_of_bluetooth_addresses[valid_index[z]];
      arrayofdistances[x][z].distance_value = rssi_distance_values[x][valid_index[z]];
      //Serial.println(valid_index[z]);
    }
  }

  //append the differences in distances to the arrays: diffInDistSet1and2 & diffInDistSet2and3
  for (int i = 0; i < count3; i++) {
    diffInDistSet1and2[i].bluetooth_address = arrayofdistances[0][i].bluetooth_address;
    diffInDistSet1and2[i].distance_value = int(arrayofdistances[1][i].distance_value) - int(arrayofdistances[0][i].distance_value);

    diffInDistSet2and3[i].bluetooth_address = arrayofdistances[0][i].bluetooth_address;
    diffInDistSet2and3[i].distance_value = int(arrayofdistances[2][i].distance_value) - int(arrayofdistances[1][i].distance_value);
  }
  sortDiffinDist();//call the sortDiffinDist function

  int counter7 = 0; //counter7 + 1 will be the number of illegal clusters
  //loop through lengthOfclusters to identify clusters of more than 5 people
  for (int x; x < num_of_device; x++) {
    if (lengthOfclusters[x] > threshold) {
      index_illegalClusters[counter7] = x;
      counter7 += 1;
    }
  }
}

//FUNCTIONS FOR computeCluster()
void sortDiffinDist() { //must call sortdiffbetweenset1and2(0, 0) first, before sortDiffinDist() and remove sortdiffbetweenset1and2(0, 0) from this function
  final_counter = 0; //represents the y parameter for sortdiffbetweenset1and2() function

  sortdiffbetweenset1and2(0, 0); //base used: person 0 (the difference in distances between set 1 and 2 will be compared to that of person 0 and those who meet the criteria will be added into the same cluster as person 0)

  for (int l = 0; l < num_of_device; l++) {
    for (int z = 0; z < num_of_device; z++) {
      if (String(allclusters[l][z]) == "0") {
        counter5 += 1;
      }
    }
    lengthOfclusters[l] = 0;
  }

  num_of_clustered_people = num_of_device - counter5;
  lengthOfclusters[0] = num_of_clustered_people;
  total_num_of_people_clustered += num_of_clustered_people;
  counter6 = 0; //tells you the number of clusters detected

  while (total_num_of_people_clustered != count3) {
    final_counter += 1;
    sortdiffbetweenset1and2(r, final_counter); //base used: person r, who does not belong to cluster 1

    counter5 = 0;
    for (int z = 0; z < num_of_device; z++) {
      if (String(allclusters[final_counter][z]) == "0") { //count the number of people in the next detected cluster
        counter5 += 1;
      }
    }

    num_of_clustered_people = num_of_device - counter5;
    total_num_of_people_clustered += num_of_clustered_people;
    lengthOfclusters[final_counter] = num_of_clustered_people;
    counter6 += 1;
  }
}

//function: finds the difference between set 1 and 2 and categorizes the people according to this difference into clusters
void sortdiffbetweenset1and2(int x, int y) { //y will be the number of the time that this function is being run so if its the first time, then y =0, then second time will be y = 1.
  String cluster1[num_of_device];

  //this line adds person x's difference in distance between set 1 and 2 to cluster 1
  cluster1[0] = diffInDistSet1and2[x].bluetooth_address; //add the bluetooth address to cluster1 (identifying feature)
  int counter = 0;
  int counter2 = 0;

  //this for loop uses person x's difference to compare with the rest of the person's differences and if they are in a +- 20 range then they are added to cluster 1 and removed from the dictionary
  for (int i = x; i < count3 - 1; i++) { //i+1 can be max of 19, which is why i can max be 18 and that's why a lot of places, the range is from 0 to (NUM_OF_HUMAN - 1)
    counter = 0;
    for (int l = 0; l < num_of_device; l++) {
      for (int z = 0; z < num_of_device; z++) {
        if (String(allclusters[l][z]) != String(diffInDistSet1and2[i + 1].bluetooth_address)) { //since there might be situations where the allclusters does contain the bluetooth address which means that it is already part of another cluster
          counter += 1;
          continue;
        } else {
          break;
        }
      }
    }
    if (counter == num_of_device * num_of_device) {
      int diff1 = diffInDistSet1and2[i + 1].distance_value;
      int diff2 = diffInDistSet1and2[x].distance_value;
      int diff3 = diffInDistSet2and3[i + 1].distance_value;
      int diff4 = diffInDistSet2and3[x].distance_value;

      if ((diff1 - diff2 < 20) && (diff1 - diff2 > -20) && (diff3 - diff4 < 20) && (diff3 - diff4 > -20)) { //criteria
        counter2 += 1;
        //add person in cluster 1
        cluster1[counter2] = diffInDistSet1and2[i + 1].bluetooth_address;
      }
    }
  }
  for (int f = 0; f < counter2 + 1; f++) { //since counter2 is like the index of cluster1, we must make f < counter2 + 1
    allclusters[y][f] = cluster1[f];
    Serial.println(cluster1[f]);
  }
  int counter6 = 0;
  if (count3 < num_of_device) { //if the no. of unique bluetooth addresses is less than the total number collected at the start (for eg. out of 5 collected, 2 are unique and 3 are repeated), then only u would have some parts of allclusters blank
    for (int c = 1; c < int(num_of_device - counter2); c++) {
      allclusters[y][counter2 + c] = "0"; //assigns empty values as 0 - should not assign it 0 when index = counter2
      counter6 += 1;
    }
  }
  //choosing the base for the next iteration of the function
  r = x;

  int counter3 = 0;
  //only if allclusters does not contain the bluetooth address of the +1, then r += 1, if it does not contain += y, then r += y
  for (int l = 0; l < num_of_device; l++) {
    for (int z = 0; z < num_of_device; z++) {
      while (String(allclusters[l][z]) == String(diffInDistSet1and2[x + 1].bluetooth_address)) { //since there might be situations where the allclusters does contain the bluetooth address which means that it is already part of another cluster
        counter3 += 1;
        break;
      }
    }
  }
  if (counter3 == 0) { //means that diffInDistSet1and2[x+1].bluetooth_address is not present in all clusters
    r += 1;
  }
  else { //this part ensures that the r value is just one greater than the previous base so that it can check for similarity in difference of distances for all the people above person r and would not miss out people in between person x and person r
    int counter4 = 0;
    for (int y = 2; y < num_of_device; y++) { //since we do not know if the person after person x has been added into a cluster or not, we choose a random maximum value of 10 where the max value of r = x + 10
      for (int l = 0; l < num_of_device; l++) {
        for (int z = 0; z < num_of_device; z++) {
          if (String(allclusters[l][z]) != String(diffInDistSet1and2[x + y].bluetooth_address)) { //since there might be situations where the allclusters does contain the bluetooth address which means that it is already part of another cluster
            counter4 += 1;
            continue;
          } else {
            break;
          }
        }
      }
      if (counter4 == 0) {
        r += y;
        break;
      }
    }
  }
}

//--------------------------------COMPUTE ANGLE OF CLUSTER WRT TO ROBOT--------------------------------
int index;
float angleOfclusters[num_of_device]; //stores the angle of each cluster wrt to the robot's origin
void computeDirection() {
  for (int x = 0; x < num_of_device; x++) {
    angleOfclusters[x] = 0;
  }

  for (int x = 0; x < counter7 + 1; x++) { //looping through all the illegal clusters
    //resetting the clusters so that the count of the number of people in each case for each cluster do not interfere with other clusters
    digitalWrite(amber, HIGH);
    int counter_1 = 0;
    int counter_2 = 0;
    int counter_3 = 0;
    int counter_4 = 0;

    int cluster_angles[num_of_device]; //stores the angles of the people of one cluster wrt to the robot's origin
    //setting everthing in cluster_angles to be 0 so that the angle added can be recognized
    for (int z = 0; z < num_of_device; z++) {
      cluster_angles[z] = 0;
    }

    //loop through each person in the cluster
    for (int y = 0; y < lengthOfclusters[x]; y++) {
      int counter_1 = 0;
      int counter_2 = 0;
      int counter_3 = 0;
      int counter_4 = 0;
      String person_bluetoothaddress = allclusters[x][y]; //the person's bluetooth address
      for (int alpha = 0; alpha < count3; alpha++) {
        if (person_bluetoothaddress == arrayofdistances[0][alpha].bluetooth_address) {
          index = alpha;
        }
      }
      //position 1 is the origin of the robot and positions 2, 3 and 4 are the subsequent positions of the robot (robot moves in a clockwise direction)

      //for Case 3: The person would be the closest to position 1 (index 0) & the diff in distance from positions 2 and 4 would be less than 125 & is either the furthest from position 3 OR the diff in distance from positions 3 and 4 is less than 15
      if ((arrayofdistances[0][index].distance_value < arrayofdistances[1][index].distance_value && arrayofdistances[0][index].distance_value < arrayofdistances[2][index].distance_value && arrayofdistances[0][index].distance_value < arrayofdistances[3][index].distance_value) && ((arrayofdistances[1][index].distance_value - arrayofdistances[3][index].distance_value <= 125) || (arrayofdistances[1][index].distance_value - arrayofdistances[3][index].distance_value >= -125))) {
        if ((arrayofdistances[2][index].distance_value > arrayofdistances[1][index].distance_value && arrayofdistances[2][index].distance_value > arrayofdistances[3][index].distance_value) || (arrayofdistances[2][index].distance_value - arrayofdistances[3][index].distance_value < 15) || (arrayofdistances[2][index].distance_value - arrayofdistances[3][index].distance_value > -15)) {
          counter_3 += 1;
        }
      } //for Case 1: The person will be the closest to either position 1 (index 0) or position 4 (index 3)
      else if (((arrayofdistances[0][index].distance_value < arrayofdistances[1][index].distance_value && arrayofdistances[0][index].distance_value < arrayofdistances[2][index].distance_value && arrayofdistances[0][index].distance_value < arrayofdistances[3][index].distance_value) && (arrayofdistances[3][index].distance_value < arrayofdistances[1][index].distance_value && arrayofdistances[3][index].distance_value < arrayofdistances[2][index].distance_value)) || ((arrayofdistances[3][index].distance_value < arrayofdistances[0][index].distance_value && arrayofdistances[3][index].distance_value < arrayofdistances[1][index].distance_value && arrayofdistances[3][index].distance_value < arrayofdistances[2][index].distance_value) && (arrayofdistances[0][index].distance_value < arrayofdistances[1][index].distance_value && arrayofdistances[0][index].distance_value < arrayofdistances[2][index].distance_value))) {
        counter_1 += 1;
      } //for Case 4:The person would be the closest to position 1 (index 0) or position 2 (index 1) & the diff in distance from positions 2 and 4 would be more than 125
      else if (((arrayofdistances[0][index].distance_value < arrayofdistances[1][index].distance_value && arrayofdistances[0][index].distance_value < arrayofdistances[2][index].distance_value && arrayofdistances[0][index].distance_value < arrayofdistances[3][index].distance_value) && (arrayofdistances[1][index].distance_value < arrayofdistances[2][index].distance_value && arrayofdistances[1][index].distance_value < arrayofdistances[3][index].distance_value)) || ((arrayofdistances[1][index].distance_value < arrayofdistances[0][index].distance_value && arrayofdistances[1][index].distance_value < arrayofdistances[2][index].distance_value && arrayofdistances[1][index].distance_value < arrayofdistances[3][index].distance_value) && (arrayofdistances[0][index].distance_value < arrayofdistances[2][index].distance_value && arrayofdistances[0][index].distance_value < arrayofdistances[3][index].distance_value))) {
        if ((arrayofdistances[1][index].distance_value - arrayofdistances[3][index].distance_value > 125) || (arrayofdistances[1][index].distance_value - arrayofdistances[3][index].distance_value < -125)) {
          counter_4 += 1;
        }
      } //for Case 2: The person would be the closest to position 3 (index 2) or position 4 (index 3)
      else if ((arrayofdistances[2][index].distance_value < arrayofdistances[0][index].distance_value && arrayofdistances[2][index].distance_value < arrayofdistances[1][index].distance_value && arrayofdistances[2][index].distance_value < arrayofdistances[3][index].distance_value) || (arrayofdistances[3][index].distance_value < arrayofdistances[0][index].distance_value && arrayofdistances[3][index].distance_value < arrayofdistances[1][index].distance_value && arrayofdistances[3][index].distance_value < arrayofdistances[2][index].distance_value)) {
        counter_2 += 1;
      }
      //finding which counter has the max value
      int max_value = max(max(max(counter_1, counter_2), counter_3), counter_4);
      //common part of angle determination for all cases
      int a = arrayofdistances[1][index].distance_value;
      int b = arrayofdistances[0][index].distance_value;
      int c = distance_robot;

      //finding which counter has the max value and whichever has the max value, we will use that case's angle determination method and find the angle & add it into a list
      if (counter_1 == max_value) {
        //use case 1 angle determination
        float A = 0.0;
        float angle = 0.0;

        float B = sq(b) + sq(c) - sq(a);
        float C = 2 * b * c;
        float D = B / C;

        A = acos(D);
        angle = 180 - (A * 180 / PI - 120);
        cluster_angles[y] = angle;
      }
      else if (counter_2 == max_value) {
        //use case 2 angle determination
        float A = 0.0;
        float angle = 0.0;

        float B = sq(b) + sq(c) - sq(a);
        float C = 2 * b * c;
        float D = B / C;

        A = acos(D);
        angle = 360 - 60 - A * 180 / PI;
        cluster_angles[y] = angle;
      }
      else if (counter_3 == max_value) {
        //use case 3 angle determination
        float A = 0.0;
        float angle = 0.0;

        float B = sq(b) + sq(c) - sq(a);
        float C = 2 * b * c;
        float D = B / C;

        A = acos(D);
        angle = A * 180 / PI - 60;
        cluster_angles[y] = angle;
      }
      else if (counter_4 == max_value) {
        //use case 4 angle determination
        float A = 0.0;
        float angle = 0.0;

        float B = sq(b) + sq(c) - sq(a);
        float C = 2 * b * c;
        float D = B / C;

        A = acos(D);
        angle = 180 + 120 + A * 180 / PI;
        cluster_angles[y] = angle;
      }

      sum_of_distances += arrayofdistances[0][index].distance_value;
    }
    float sum_of_angles = 0;

    //finding the mean of the angles added into the list
    for (int i = 0; i < lengthOfclusters[x]; i ++) {
      sum_of_angles += cluster_angles[i];
      Serial.println(cluster_angles[i]);
    }

    float angle_of_cluster_wrt_robot = sum_of_angles / lengthOfclusters[x];
    float distance_of_cluster_wrt_robot = sum_of_distances / lengthOfclusters[x];
    //append the angle into a list
    angleOfclusters[x] = angle_of_cluster_wrt_robot;

    //checking for obstacles in its radius within the distance of the cluster so that it experiences no obstacles while the robot moves towards the cluster
    for (int angle = 0; angle < 180; angle++) {
      ultrasonicServo.write(angle);
      delay(15);
      getSonarDistance();
      while (distance > 0 && distance < distance_of_cluster_wrt_robot - 10) { //subtracting 10 just for buffer
        stopMovement();
      }
      timeTaken = 0;
      distance = 0;
    }

    float angle;
    if (distance >= distance_of_cluster_wrt_robot - 10) {
      //moving of the robot to the illegal cluster
      int delay_time_moveForward = (distance_of_cluster_wrt_robot / 100) * (delay_time_100cm) - 15;

      int delay_time_rotate = (angle_of_cluster_wrt_robot / 60) * (delay_time_60degrees) - 15;

      digitalWrite(red, HIGH);
      moveLeft();
      delay(delay_time_rotate);
      moveForward();
      delay(delay_time_moveForward);

      /*------------------------------------------------SOUND BUZZER SECTION-------------------------------------------------
        -----------------------------------------------------------------------------------------------------------------------
        -----------------------------------------------------------------------------------------------------------------------
        -----------------------------------------------------------------------------------------------------------------------
        -----------------------------------------------------------------------------------------------------------------------*/


      digitalWrite(red, LOW);
      digitalWrite(amber, LOW);

      //moving of the robot back to the original location such that it faces the same direction as before
      moveBackward();
      delay(delay_time_moveForward);
      moveRight();
      delay(delay_time_rotate);
    }
  }
}

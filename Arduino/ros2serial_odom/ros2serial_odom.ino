void setup() {
  Serial.setTimeout(10);
  Serial.begin(115200);
}

unsigned long last_update = 0;

double position_x = 0;
double position_y = 0;
double position_z = 0;
double rotation_z = 0;
double linear_x  = 0;
double linear_y  = 0;
double linear_z  = 0;
double anguler_x = 0;
double anguler_y = 0;
double anguler_z = 0;

int split(char* str, char regex, char** buf, size_t buflen){
    int strlength = strlen(str);
    int length = 1;
    buf[0] = str;
    for(int i = 1; i < strlength; i++){
        if(str[i] == regex){
            str[i] = '\0';
            buf[length] = &str[i + 1];
            length++;
            if(buflen <= length)break;
        }
    }
    return length;
}

void loop() {
  char buff[128] = "";
  if(Serial.readBytesUntil('\n',buff,sizeof(buff)) > 0){
    Serial.print("rceive:");Serial.println(buff);
    char* arr[6];
    if(split(buff,',',arr,6) == 6){
      linear_x  = atof(arr[0]);
      linear_y  = atof(arr[1]);
      linear_z  = atof(arr[2]);
      anguler_x = atof(arr[3]);
      anguler_y = atof(arr[4]);
      anguler_z = atof(arr[5]);

      Serial.print("rceive:");
      Serial.print(linear_x);Serial.print(' ');
      Serial.print(linear_y);Serial.print(' ');
      Serial.print(linear_z);Serial.print(' ');
      Serial.print(anguler_x);Serial.print(' ');
      Serial.print(anguler_y);Serial.print(' ');
      Serial.print(anguler_z);Serial.println();
    }
  }

  unsigned long now_time = millis();
  double dt = (double)(now_time - last_update) * 0.001;
  last_update = now_time;

  position_x += linear_x * dt;
  position_y += linear_y * dt;
  position_z += linear_z * dt;
  rotation_z += anguler_z * dt;
    
  Serial.print(position_x);Serial.print(' ');
  Serial.print(position_y);Serial.print(' ');
  Serial.print(position_z);Serial.print(' ');
  Serial.print(0.0);                  Serial.print(' ');
  Serial.print(0.0);                  Serial.print(' ');
  Serial.print(sin(0.5 * rotation_z));Serial.print(' ');
  Serial.print(cos(0.5 * rotation_z));Serial.println();
    
  delay(1000);
}

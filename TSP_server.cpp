#include<iostream>
#include<iomanip>
#include <string>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
//   library for Windows socket
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
// Thread for image proccessing
#include <pthread.h>
#include <time.h>
using namespace std;

#define MAX_CLIENT 20
#define real_user 6
#define andport 22041
#define droneport 22042
const double pi = 3.1415926535897932384626433832795028841971693993751;

double completed[10], cost = 0;
int user;   // service requesting user number (target point number)
int* course; // global variable about TSP Path
int coursenum = 0;   // location of course array

struct Target {      // Latitude and Longitude per each user(user numbering)
   const char* pointNum;   // user number requesting while 3 min(target point number)
   double latitude;
   double longitude;
};

Target targetdata[21] = {{"0", 35.132882, 129.106199}, {"1", 35.133208, 129.105933}, {"2", 35.133208, 129.106063},
{"3", 35.133208, 129.106193}, {"4", 35.133208, 129.106323}, {"5", 35.133208, 129.106452},
{"6", 35.133057, 129.105933}, {"7", 35.133057, 129.106063}, {"8", 35.133057, 129.106193},
{"9", 35.133057, 129.106323}, {"10", 35.133057, 129.106452}, {"11", 35.133676, 129.105952},
{"12", 35.133676, 129.106082}, {"13", 35.133676, 129.106212}, {"14", 35.133676, 129.106342},
{"15", 35.133676, 129.106472}, {"16", 35.132525, 129.105952}, {"17", 35.132525, 129.106082},
{"18", 35.132525, 129.106212}, {"19", 35.132525, 129.106342}, {"20", 35.132525, 129.106472}};
// base point and allocated target point(Drone zone) GPS point


string snddata;   // string to send Client

// to save user requesting GPS drone zone point information from Android app of User(client)
Target from_user[real_user];

// GPS point comparing function prototype
double toRad(double degree);      // degree to radian
double calculateDistance(double lat1, double long1, double lat2, double long2);   // to calculate GPS data

// TSP algorithm function Prototype
void takeInput(int points, double cordinates[real_user][real_user]);
double least(int c, int points, double cordinates[real_user][real_user]);
void mincost(int city, int points, double cordinates[real_user][real_user], int* course);

// Scoket connection and message function Prototype
string makemsg(string snddata, struct Target targetpoint[real_user]);
void* t_function(void* data);      // for thread to connect multiple Android user

int client_index = 0;      // current calling service user(Android app) number
int current_user = 0;
int main()
{
   /* service process
   get user wanting drone zone through Android app -> put requested drone zone data int TSP algorithm
   -> calculating TSP path, and send shortest visiting path data to Drone client(Pixhawk-Pi) */
   cout << "Start Drone Delivery System\n";

   int i, j;   // value for loop

   for(int i=0; i<real_user; i++){
      from_user[i].pointNum = {0};
      from_user[i].latitude = 0;
      from_user[i].longitude = 0;
   }
   // Home Base point
   from_user[0].pointNum = targetdata[0].pointNum;
   from_user[0].latitude = targetdata[0].latitude;
   from_user[0].longitude = targetdata[0].longitude;
   client_index++;

   // Variables for Socket
   // For android socket
   int server_sock, client_sock;
   struct sockaddr_in server_addr, client_addr;
   int client_addr_size = sizeof(client_addr);
   server_addr.sin_addr.s_addr = INADDR_ANY;
   server_addr.sin_family = AF_INET;
   server_addr.sin_port = htons(andport);   // For android socket
   pthread_t thread_client[MAX_CLIENT];

   // For Drone socket
   int server_sock2, client_sock2;
   struct sockaddr_in server_addr2, client_addr2;
   int client_addr_size2 = sizeof(client_addr2);
   server_addr2.sin_addr.s_addr = INADDR_ANY;
   server_addr2.sin_family = AF_INET;
   server_addr2.sin_port = htons(droneport);   // For Drone(Pixhawk-Pi) socket

   // Android socket connection start
   if ((server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)      // socket create
   {
      cout << "socket create error\n";
      return -1;
   }

   int on = 1;
   if (setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0)
   {
      cout << "socket option set error\n";
      return -1;
   }

   if (bind(server_sock, (struct sockaddr*) & server_addr, sizeof(server_addr)) < 0)      // bind
   {
      cout << "bind error\n";
      return -1;
   }

   if (listen(server_sock, 5) < 0)      // listen
   {
      cout << "listen error\n";
      return -1;
   }

   cout << "\nWaiting Client...\n\n";   // ready to connect Android client

   while (1)      // android socket connection
   {
      client_sock = accept(server_sock, (struct sockaddr*) & client_addr, (socklen_t*)& client_addr_size);
      current_user++;
      cout << "==================================================\nClient Addr " << inet_ntoa(client_addr.sin_addr)
     << "is Waiting\nCurrent waiting User : " << current_user << "\nPort : " << ntohs(client_addr.sin_port)
     << "\n==================================================\n";

      if (client_sock < 0)
      {
         cout << "accept error\n";
      }

      if (current_user == MAX_CLIENT)
      {
         cout << "client accept full(max client count : " << MAX_CLIENT << "\n";
         close(client_sock);
         continue;
      }

      if (pthread_create(&thread_client[current_user], NULL, t_function, (void*)& client_sock) != 0)
      {
         cout << "Thread create error\n";
         close(client_sock);
         continue;
      }
      if (client_index >= 5){   // client number over 10 or waiting time over 180 sec then server socket close
         break;
      }
   }
   // Android client socket connection finish
   while(client_index != 6){
      if(client_index==6) break;
   }

   cout << "\nCurrent Client : " << client_index-1 << "\n";
   close(server_sock);

   // TSP algorithm
   double dist[real_user][real_user];   // To save distance calculating result from Android Client

   for (i = 0; i < real_user; i++) {      // make 2dimension array by using calculateDistance function
      for (j = 0; j < real_user; j++) {
         if (i == j) dist[i][j] = 0;
         else {
            dist[i][j] = calculateDistance(from_user[i].latitude,
            from_user[i].longitude, from_user[j].latitude, from_user[j].longitude);
         }
      }
   }

   course = new int[real_user + 1];      // course dynamic memory allocation
   takeInput(real_user, dist);

   cout << "\n\nDelivery Client order => ";
   mincost(0, real_user, dist, course); //passing 0 because starting vertex

   cout << "\nDelivery Drone Zone Path";
    for(i=0; i < coursenum+1 ; i++){
        cout << " => " << from_user[course[i]].pointNum;

        snddata.append(to_string(from_user[course[i]].latitude));
        snddata.append("/");
        snddata.append(to_string(from_user[course[i]].longitude));
        snddata.append("/");
    }

   cout << fixed;
   cout.precision(2);
   cout << "\n\nMinimum cost is " << cost << "m\n";

   const char* msg = snddata.c_str();      // convert snddata to char type message
   // TSP algorithm finish and making path and message done


   // Drone socket connection start
   if ((server_sock2 = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)      // socket create
   {
      cout << "socket create error\n";
      return -1;
   }

   if (bind(server_sock2, (struct sockaddr*) & server_addr2, sizeof(server_addr2)) < 0)      // bind
   {
      cout << "bind error\n";
      return -1;
   }

   if (listen(server_sock2, 5) < 0)      // listen
   {
      cout << "listen error\n";
      return -1;
   }

   cout << "\n\nWaiting Drone(Pi) Client...\n\n";      // raedy to connect Drone socket

   client_sock2 = accept(server_sock2, (struct sockaddr*) & client_addr2, (socklen_t*)& client_addr_size2);
   if (client_sock2 < 0)
   {
      cout << "accept error\n";
   }

   if (write(client_sock2, msg, BUFSIZ) <= 0)
   {
      printf("Client %d close\n", client_sock2);
      close(client_sock2);
   }

   cout << "Send Message : " << msg << "\n\n";

   cout << "\nSucess! Sending Path message to Drone.\n";

   sleep(3);
//   close(client_sock2);
   close(server_sock2);
   // end of Socket connection
   cout <<  "\nDisconnect!! Bye~!\n\n";   // Drone client socket connection finish

   return 0;
}

double toRad(double degree) {
   return degree / 180 * pi;
}

// Indoor virture location(x, y) lat = Latitude(x), long = longitude(y)
double calculateDistance(double lat1, double long1, double lat2, double long2) {
   double dist;
   dist = sin(toRad(lat1)) * sin(toRad(lat2)) + cos(toRad(lat1)) * cos(toRad(lat2)) * cos(toRad(long2 - long1));
   dist = acos(dist);
   //   dist = (6371 * pi * dist) / 180;
   //   got dist in radian, no need to change back to degree and convert to rad again.
   dist = 6371000 * dist;
   return dist;
}

void takeInput(int points, double cordinates[real_user][real_user])      // points : service requesting user number, cordinates : TSP 2dimension array by using point
{
   int i, j;
   int n = points;
    cout << fixed;
   cout.precision(2);

    cout << "\nThe number of points(Included Home Point) : " << points << "\n";

   cout << "\nTSP Cost Matrix\n";
   for (i = 0; i < n; i++) {
      for (j = 0; j < n; j++) {
         cout.setf(ios::right);
         cout << setw(8) << cordinates[i][j] << "m";
      }
      cout << "\n";

      completed[i] = 0;
   }
}

double least(int c, int points, double cordinates[real_user][real_user])
{
   int i, nc = 999;
   int min = 999;
   double kmin;
   int n = points;

   for (i = 0; i < n; i++)
   {
      if ((cordinates[c][i] != 0) && (completed[i] == 0))
         if (cordinates[c][i] + cordinates[i][c] < min)
         {
            min = cordinates[i][c] + cordinates[c][i];
            kmin = cordinates[c][i];
            nc = i;
         }
   }

   if (min != 999)
      cost += kmin;

   return nc;
}

void mincost(int city, int points, double cordinates[real_user][real_user], int* course)
{
   int i, ncity;

   completed[city] = 1;

   cout << city << " ---> ";

   course[coursenum] = city;
   coursenum++;

   ncity = least(city, points, cordinates);

   if (ncity == 999)
   {
      ncity = 0;
      cout << ncity;
      cost += cordinates[city][ncity];

      return;
   }

   mincost(ncity, points, cordinates, course);
}

void* t_function(void* arg)
{
   cout << fixed;
   cout.precision(6);

   int client_sock = *((int*)arg);
   pid_t pid = getpid();      // process id
   pthread_t tid = pthread_self();  // thread id

   cout << "\nAndroid User Conecction Successfully!!\n";
   char clientpoint[BUFSIZ] = "\0";
   int i=0;

   memset(clientpoint, 0x00, BUFSIZ);
   // compare and copy data to put and calculate TSP algorithm
   if (client_index < 6) {
      (read(client_sock, clientpoint, BUFSIZ));
      i = client_index;   // to save client index while client thraed function run
      for (int cnt = 0; cnt < 21; cnt++) {
         if (strcmp(clientpoint, targetdata[cnt].pointNum) == 0) {   // comparing from_user and targetdata
            from_user[i].pointNum = targetdata[cnt].pointNum;
            from_user[i].latitude = targetdata[cnt].latitude;
            from_user[i].longitude = targetdata[cnt].longitude;
            char buf[512] = {"Booking complete! Thank you!"};
            (write(client_sock, buf, sizeof(buf)));
            client_index++;   // add current service requesting client index
            break;
         }
      }
   }
   else {
      read(client_sock, clientpoint, BUFSIZ);
      char buf[512] = {"Reservation is fulled! Wait for next delivery"};
      (write(client_sock, buf, sizeof(buf)));
      close(client_sock);
      }

   cout << "*****Client " << i << "*****\nDrone Zone : " << from_user[i].pointNum
   << ",   Latitude : " << from_user[i].latitude << ",   Longitude : " << from_user[i].longitude << "\n";
   cout << "pid : " << (unsigned int)pid << ", tid : " << (unsigned int)tid << "\n";
   cout << "Client " << i << " close\n\n";
   close(client_sock);
}

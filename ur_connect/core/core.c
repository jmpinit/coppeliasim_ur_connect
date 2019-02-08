#define LUA_LIB

#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <string.h>

#include "lua.h"
#include "lauxlib.h"

#define DEBUG

typedef struct RobotPose {
  int base;
  int shoulder;
  int elbow;
  int wrist1;
  int wrist2;
  int wrist3;
} RobotPose;

typedef struct ServerArgs {
  char* ip;
  int port;
} ServerArgs;

const int CMD_INACTIVE = 0xff;
const int MULT_JOINTSTATE = 1000000;

// The internal controller which sends commands to the joint servos runs at
// 125 Hz - Optimizing the Universal Robots ROS driver, Anderson, Thomas Timm
#define CONTROL_RATE 125

pthread_t threadServer;
bool serverRunning = false;

uint8_t activeCommand = CMD_INACTIVE;
RobotPose currentPose = { 0 };
pthread_mutex_t lockCurrentPose;
RobotPose sensedPose = { 0 };
pthread_mutex_t lockSensePose;

bool haveSensedPose = false;

void die(const char *msg) {
  perror(msg);
  exit(1);
}

void *run_server(void *args) {
  ServerArgs* serverArgs = args;

  struct sockaddr_in serv_addr;

  // Server configuration
  serv_addr.sin_family = AF_INET; // IPv4
  serv_addr.sin_port = htons(serverArgs->port); // (host to network byte order)
  serv_addr.sin_addr.s_addr = inet_addr(serverArgs->ip);

#ifdef DEBUG
  printf("Starting server at %s:%d\n", inet_ntoa(serv_addr.sin_addr), ntohs(serv_addr.sin_port));
#endif

  int sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
#ifdef DEBUG
    perror("Error opening socket");
#endif
    return NULL;
  }

  // Reuse address if necessary
  int enable = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
#ifdef DEBUG
    perror("setsockopt(SO_REUSEADDR) failed");
#endif
  }

  // Bind the address for the server
  if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
#ifdef DEBUG
    perror("Error on binding");
#endif
    return NULL;
  }

  // Set to nonblocking so the thread doesn't get stuck if it needs to exit
  int flags = fcntl(sockfd, F_GETFL, 0);
  fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

  // Start listening for connections
  listen(sockfd, 1);
#ifdef DEBUG
  printf("Listening at %s:%d\n", inet_ntoa(serv_addr.sin_addr), ntohs(serv_addr.sin_port));
#endif

  struct sockaddr_in cli_addr = { 0 };
  socklen_t clilen = sizeof(cli_addr);
  int clientSockFd;

  while (true) {
    // Accept a client connection

    bool clientAccepted = false;

    while (!clientAccepted) {
      clientSockFd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);

      if (clientSockFd < 0) {
        if (errno != EAGAIN) {
          die("Error on accept");
        }
      } else {
        clientAccepted = true;
      }

      if (!serverRunning) {
        // Server has been interrupted
        goto server_exit_no_client;
      }

      usleep(1000);
    }

    char* clientAddress = inet_ntoa(cli_addr.sin_addr);
    int clientPort = ntohs(cli_addr.sin_port);

#ifdef DEBUG
    printf("Connection from %s port %d\n", clientAddress, clientPort);
#endif

    assert(sizeof(int) == 4); // Robot expects 32 bit ints

    const int msgLen = 7;
    int32_t buffer[msgLen] = { 0 };

    while (true) {
      if (!serverRunning) {
        goto server_exit;
      }

      pthread_mutex_lock(&lockCurrentPose);
      buffer[0] = htonl(currentPose.base);
      buffer[1] = htonl(currentPose.shoulder);
      buffer[2] = htonl(currentPose.elbow);
      buffer[3] = htonl(currentPose.wrist1);
      buffer[4] = htonl(currentPose.wrist2);
      buffer[5] = htonl(currentPose.wrist3);
      buffer[6] = htonl(activeCommand);
      pthread_mutex_unlock(&lockCurrentPose);

      // Send a message to the client
      write(clientSockFd, buffer, sizeof(buffer));

      // Read a message from the client
      const int replyLength = sizeof(buffer) - sizeof(int) + 1;
      int messageLength = read(clientSockFd, buffer, replyLength);

      if (messageLength > 0) {
        // Ensure a full reply was received
        if (messageLength == replyLength) {
          // Copy it
          pthread_mutex_lock(&lockSensePose);
          sensedPose.base = ntohl(buffer[0]);
          sensedPose.shoulder = ntohl(buffer[1]);
          sensedPose.elbow = ntohl(buffer[2]);
          sensedPose.wrist1 = ntohl(buffer[3]);
          sensedPose.wrist2 = ntohl(buffer[4]);
          sensedPose.wrist3 = ntohl(buffer[5]);
          pthread_mutex_unlock(&lockSensePose);

          haveSensedPose = true;
        }
      }

      if (activeCommand == 1) {
        // The robot stops after a movej (command 1) so we should wait for an
        // affirmative instruction that movement should be continued
        activeCommand = CMD_INACTIVE;
      }

      usleep(1000000 / CONTROL_RATE);
    }
  }

server_exit:
  // Write one last message that tells the robot to stop and exit the control script
  {
    int32_t poisonPill[7] = {0};
    write(clientSockFd, poisonPill, sizeof(poisonPill));
  }

  close(clientSockFd);
server_exit_no_client:
  close(sockfd);
#ifdef DEBUG
  printf("Server shut down\n");
#endif

  free(serverArgs->ip);
  free(serverArgs);

  return NULL;
}

static int ur_connect_core_start_server(lua_State *L) {
  if (!lua_isstring(L, 1)) {
    luaL_typerror(L, 1, "string");
    return 0;
  }

  if (!lua_isnumber(L, 2)) {
    luaL_typerror(L, 2, "number");
    return 0;
  }

  serverRunning = true;

  int port = lua_tonumber(L, 2);
  const char* ipAddress = lua_tostring(L, 1);
  lua_pop(L, 2);

  ServerArgs *serverArgs = malloc(sizeof *serverArgs);
  serverArgs->ip = calloc(1, strlen(ipAddress) + 1);
  serverArgs->port = port;
  strcpy(serverArgs->ip, ipAddress);

  activeCommand = CMD_INACTIVE;

  if (pthread_create(&threadServer, NULL, run_server, serverArgs)) {
    lua_pushstring(L, "Unable to start server thread");
    lua_error(L);
    return 0;
  }

#ifdef DEBUG
  printf("Server thread created\n");
#endif

  return 0;
}

static int ur_connect_core_stop_server(lua_State *L) {
  if (!serverRunning) {
    return 1;
  }

#ifdef DEBUG
  printf("Stopping server\n");
#endif

  haveSensedPose = false;
  serverRunning = false;
  pthread_join(threadServer, NULL);
  return 0;
}

static int ur_connect_core_get_pose(lua_State *L) {
  if (!haveSensedPose) {
    // No value to report yet
    lua_pushnil(L);
    return 1;
  }

  pthread_mutex_lock(&lockSensePose);
  float values[6] = {
    (float)sensedPose.base / MULT_JOINTSTATE,
    (float)sensedPose.shoulder / MULT_JOINTSTATE,
    (float)sensedPose.elbow / MULT_JOINTSTATE,
    (float)sensedPose.wrist1 / MULT_JOINTSTATE,
    (float)sensedPose.wrist2 / MULT_JOINTSTATE,
    (float)sensedPose.wrist3 / MULT_JOINTSTATE,
  };
  pthread_mutex_unlock(&lockSensePose);

  for (int i = 0; i < 6; i++) {
    lua_pushnumber(L, values[i]); // Value
  }

  return 6;
}

static int ur_connect_core_update_pose(lua_State *L) {
  if (!lua_istable(L, 1)) {
    luaL_typerror(L, 1, "table");
    return 0;
  }

  if (lua_objlen(L, 1) != 6) {
    lua_pushstring(L, "Expected 6 values");
    lua_error(L);
    return 0;
  }

  if (!lua_isnumber(L, 2)) {
    luaL_typerror(L, 2, "number");
    return 0;
  }

  int newCommand = lua_tonumber(L, 2);

  // Read the joint angle table

  float values[6] = {0};

  for (int i = 0; i < 6; i++) {
    lua_pushnumber(L, i + 1); // Key
    lua_gettable(L, 1); // Pops key, pushes value
    values[i] = lua_tonumber(L, -1);
    lua_pop(L, 1); // Pop value
  }

  // Pop arguments
  lua_pop(L, 2);

  #ifdef DEBUG
  if (newCommand != activeCommand) {
    printf("Active command changed to %d from %d\n", newCommand, activeCommand);
  }
  #endif

  pthread_mutex_lock(&lockCurrentPose);
  currentPose.base = values[0] * MULT_JOINTSTATE;
  currentPose.shoulder = values[1] * MULT_JOINTSTATE;
  currentPose.elbow = values[2] * MULT_JOINTSTATE;
  currentPose.wrist1 = values[3] * MULT_JOINTSTATE;
  currentPose.wrist2 = values[4] * MULT_JOINTSTATE;
  currentPose.wrist3 = values[5] * MULT_JOINTSTATE;
  activeCommand = newCommand;
  pthread_mutex_unlock(&lockCurrentPose);

  return 0;
}

static int ur_connect_core_get_assigned_ips(lua_State *L) {
  int fd = socket(AF_INET, SOCK_DGRAM, 0);

  struct ifreq ifr;
  ifr.ifr_addr.sa_family = AF_INET;
  strncpy(ifr.ifr_name, "en0", IFNAMSIZ - 1);

  ioctl(fd, SIOCGIFADDR, &ifr);

  close(fd);

  const char* ipAddr = inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
  lua_pushstring(L, ipAddr);

#ifdef DEBUG
  printf("get_assigned_ips called and returned %s\n", ipAddr);
#endif

  return 1;
}

static const struct luaL_Reg ur_connect_core_funcs[] = {
  { "start_server", ur_connect_core_start_server },
  { "stop_server", ur_connect_core_stop_server },
  { "get_pose", ur_connect_core_get_pose },
  { "update_pose", ur_connect_core_update_pose },
  { "get_assigned_ips", ur_connect_core_get_assigned_ips },
  { NULL, NULL },
};

static int hook_gc(lua_State *L) {
#ifdef DEBUG
  printf("GC hook called\n");
#endif
  serverRunning = false;
  pthread_join(threadServer, NULL);
  return 0;
}

static void stackDump(lua_State *L) {
  int i;
  int top = lua_gettop(L);
  for (i = 1; i <= top; i++) { /* repeat for each level */
    int t = lua_type(L, i);
    switch (t) {
      case LUA_TSTRING: /* strings */
        printf("\"%s\"", lua_tostring(L, i));
        break;

      case LUA_TBOOLEAN: /* booleans */
        printf(lua_toboolean(L, i) ? "true" : "false");
        break;

      case LUA_TNUMBER: /* numbers */
        printf("%g", lua_tonumber(L, i));
        break;

      default: /* other values */
        printf("%s", lua_typename(L, t));
        break;
    }
    printf("  "); /* put a separator */
  }
  printf("\n"); /* end the listing */
}

LUALIB_API int luaopen_ur_connect_core(lua_State *L) {
  luaL_register(L, "ur_connect_core", ur_connect_core_funcs); // pushes new table with lib functions

  lua_pushstring(L, "gc_hook");
  lua_newuserdata(L, 1);

  luaL_newmetatable(L, "gc_hook_meta"); // pushes new table
  lua_pushstring(L, "__gc");
  lua_pushcfunction(L, hook_gc);
  lua_settable(L, -3); // Sets __gc for metatable. Pops key and value off stack
  lua_setmetatable(L, -2); // Sets metatable for userdata on stack. Pops table from the stack

  // Sets gc_hook key to userdata value on library table
  lua_settable(L, -3); // pops key and value off stack

#ifdef DEBUG
  printf("Configured GC hook\n");
#endif
  stackDump(L);

  return 1;
}

#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#define PORT 5300
#define BUFLEN 128


/* TODO: implement proper error handling for Oden. */
void die_with_error(char *s) {
  perror(s);
  exit(1);
}

typedef struct {
  float w;
  float x;
  float y;
  float z;
} quaternion_t;

/* We are receiver */
int main(int argc, char *argv[]) {
  struct sockaddr_in si_receiver, si_sender;
  int socket; 
  socklen_t slen_sender = sizeof(si_sender);
  char buf[BUFLEN];
  quaternion_t quaternion = {0.0f, 0.0f, 0.0f, 0.0f};

  if ((socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    die_with_error("Failed to create UDP socket");
  }

  memset((char *) &si_receiver, 0, sizeof(si_receiver));
  si_receiver.sin_family = AF_INET;
  si_receiver.sin_port = htons(PORT);
  si_receiver.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(socket, (struct sockaddr*) &si_receiver, sizeof(si_receiver)) == -1) {
    die_with_error("Failed to bind UDP socket");
  }
    
  while (1) {
    if (recvfrom(s, buf, BUFLEN, 0, (struct sockaddr*) &si_sender, &slen_sender) == -1) {
      die_with_error("Failed recvfrom()");
    } else {
      printf("Received packet from %s:%d\nData: %s\n", inet_ntoa(si_sender.sin_addr), ntohs(si_sender.sin_port), buf);
      sscanf(buf, "%a %a %a %a", &quaternion.w, &quaternion.x, &quaternion.y, &quaternion.z);
      printf("quaternion: %f %f %f %f\n\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    }
  }
  
  close(s);
  return 0;
}
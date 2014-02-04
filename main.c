#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/time.h>
#include <netinet/in.h>
#include "helper.h"
#include <pthread.h>
#include "ur5lib.h"
#include <stdbool.h>
#include <math.h>
#include "../libs/robotinterface.h"
#include "../libs/Configuration.h"
#include "../libs/errorCodes.h"
#include "../libs/interrupt_utils.h"
#include "../libs/base_utils.h"
#include "../libs/startup_utils.h"

pthread_mutex_t connect_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t quit_programm_mutex = PTHREAD_MUTEX_INITIALIZER;

int client_fd, server_fd;
bool connected=false;
//bool quit_program;
#define TIMEOUT_TOLERANCE 3
#define NOT_INITIALIZED 0

void * start_tcp_server(void * args){
    int conn;
    struct connection_data *server = (struct connection_data *) args;
    printf("%s:%d\n", server->ip_addr_s, server->port);
    fflush(stdout);
    create_socket(&server->sockfd);
    bind_socket(server);

    server->clilen = sizeof(server->cli_addr);

    while(quit_program != true){
        printf("waiting for client to connect...");
        client_fd = accept(server->sockfd, (struct sockaddr  *) &server->cli_addr, &server->clilen);
        if (client_fd < 0){
            printf("client %s could not connected to server\n", inet_ntoa(server->cli_addr.sin_addr));
            continue;
        }

        pthread_mutex_lock(&connect_mutex);
        connected=1;
        conn=1;
        pthread_mutex_unlock(&connect_mutex);

        printf("connected: %s\n", inet_ntoa(server->cli_addr.sin_addr));
        while(conn == false){
            pthread_mutex_lock(&connect_mutex);
            conn=connected;
            pthread_mutex_unlock(&connect_mutex);
        }
        close(client_fd);
        puts("\rdisconnect from client");
    }
    pthread_mutex_lock(&connect_mutex);
    connected=false;
    pthread_mutex_unlock(&connect_mutex);
    printf("shutdown server ...\n");
    close(server->sockfd);
    pthread_exit(NULL);
}

int main(int argc, char **argv)
{
    printf("ur5 Server Start\n");
//    JointVector tcp_home= {0.0, -191.0, 600.0, 0.0, -2.2211, -2.2211};
    JointVector joint_home_rad={0.0000, -1.5708, 0.0000, -1.5708, 0.0000, 0.0000};
    JointVector joint_home={0.0000, -90, 0.0000, -90, 0.0000, 0.0000};
    JointVector init_two ={-6.5018, -95.3469, -29.8123, -194.7758, -59.0822, -62.3032};
    JointVector init_right_side = {0.309199, -1.240009, 0.323497, -1.039543, 0.541075, 0.486453};
    JointVector initialP_rad = {-0.330294, -1.881878, -0.290919, -2.243194, -0.660115, -0.704284};

    JointVector viereck[4] = {{43.24,   -140.88,    -30.05, -12.83, 129.28, -0.27},
                              {130.67,  -140.87,    -30.05, -12.75, 44.08,  -0.27},
                              {130.67,  -187.90,    6.46,   -1.22,  43.35,  -0.27},
                              {43.24,   -187.90,    -30.05, 3.87,   100.32, -0.27}
                             };
    JointVector PosOne= {36.12,   -112.88,    -8.78, -24.53, 88.39, 16.64};
    int i,h,n=0;
    for(i=0; i<4;i++){
        for(h=0;h<6;h++){
            viereck[i][h]= deg_to_rad(viereck[i][h]);
        }
    }
    for(i=0;i<MSG_BUFFER_SIZE;i++) msg_buffer[i].text = msg_text_buffers[i];

//    JointVector initialP={-30.4347, -132.2621, -40.8718, -149.8236, -59.1151, -62.2781};

//    int i;
//    printf("joint_home ={");
//    for(i=0; i<6; i++){
//        joint_home_rad[i]= deg_to_rad(joint_home[i]);
//        joint_home[i] = rad_to_deg(joint_home_rad[i]);
//        printf("%3.4f%s",joint_home_rad[i], i<5 ? ", " : "};");
////    }
//    MovePTPPacket move_ptp_packet;

    for(i=0; i<6; i++){
        PosOne[i]= deg_to_rad(PosOne[i]);}
//    move_ptp(joint_home_rad, PosOne , &move_ptp_packet);
//    return 0;
    struct connection_data server;

    fd_set masterfds, readfds;
    struct timeval timeout,send_time_start, send_time_end;

    int rc;
    pthread_t tcp_server_thread;

    MovePTPPacket move_ptp_packet;
    server.initialize_direction = -1.0;
    read_args(argc, argv, &server);
    initialize_direction = server.initialize_direction;
    printf("initialize direction: %f", initialize_direction);


    rc = pthread_create(&tcp_server_thread, NULL, &start_tcp_server, &server);
    if(rc < 0){
        quit_program=true;
    }
    int connection_timeout_count=0;
    pva_packet.header.protocol_id=PVA_PACKET_ID;
    p_packet.header.protocol_id=PPACKET_ID;
    //--------------------------- Init Robot -------------------------------------------------------
    configuration_load();
    open_interface();
    power_up();
    set_wrench();
    if(!initialize(0)){
        puts("could not initialize robot");
        exit(EXIT_FAILURE);
    }

    settle();

    setup_sigint();
    setup_sigpipe();

    memcpy(&last_pva_packet, &pva_packet,sizeof(PVAPacket));
    for(i=0;i<6;i++) last_pva_packet.acceleration[i] = 0.0;
    for(i=0;i<6;i++) last_pva_packet.velocity[i] = 0.0;
    bool timeout_flag=false;
    char *buff[sizeof(PVAPacket)];

    //------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------

//    for(pva_packet.header.cycle_number =0; quit_program == false; pva_packet.header.cycle_number++) {
//        robotinterface_read_state_blocking();
//        pthread_mutex_lock(&connect_mutex);
//        if(connection_timeout_count > TIMEOUT_TOLERANCE){
//            connection_timeout_count=0;
//            connected=false;
//        }
//        //--------------------------- Stuff roboter does -------------------------------------------------------

//        robotinterface_get_actual(servo_packet.position, servo_packet.velocity);

//        //------------------------------------------------------------------------------------------------------
//        //------------------------------------------------------------------------------------------------------



//        //---------------------------- send  -------------------------------------------------------------------
//        if(connected){
//            gettimeofday(&send_time_start,0);
//            if(send(client_fd, (char*) &pva_packet , sizeof(pva_packet),0) < 1){
//                connected=false;
//            }
//        }

//        //------------------------------------------------------------------------------------------------------
//        //------------------------------------------------------------------------------------------------------


//        //---------------------------- recieve -----------------------------------------------------------------
//        timeout_flag=false;
//        if(connected){
//            FD_ZERO(&masterfds);
//            FD_SET(client_fd, &masterfds);
//            memcpy(&readfds, &masterfds, sizeof(fd_set));
//            gettimeofday(&send_time_end,0);
//            timeval_diff(&timeout, &send_time_start, &send_time_end);
//            timeout.tv_usec = 6000 - timeout.tv_usec;
//            if (select(client_fd +1, &readfds, NULL, NULL, &timeout) < 0) {
//                pthread_mutex_lock(&connect_mutex);
//                connected=false;
//                pthread_mutex_unlock(&connect_mutex);
//            }

//            if(FD_ISSET(client_fd, &readfds)){
//                n=recv(client_fd, buff, sizeof(buff), 0);
//                if (n < 1 || n != sizeof(PVAPacket)){
//                    pthread_mutex_lock(&connect_mutex);
//                    connected=false;
//                    pthread_mutex_unlock(&connect_mutex);
//                }
//            }else{
//               connection_timeout_count++;
//               if(connection_timeout_count< TIMEOUT_TOLERANCE)
//               timeout_flag=true;
//            }
//            if(!timeout_flag && connected){
//                memcpy(&pva_packet, buff, n);
//                if(connection_timeout_count) connection_timeout_count--;
//            }
//        }

//        //------------------------------------------------------------------------------------------------------
//        //------------------------------------------------------------------------------------------------------




//        //---------------------------- do stuff with data ------------------------------------------------------


//        // TODO check for errors-----
//        //------------------------------------------------------------------------------------------------------


//        if(!timeout_flag && connected){
//            // save pva to last if something went wrong;
//            memcpy(&last_pva_packet, &pva_packet, sizeof(PVAPacket);
//        }


//        //------------------------------------------------------------------------------------------------------
//        //------------------------------------------------------------------------------------------------------




//        //--------------------------- Stuff roboter does -------------------------------------------------------
//        if(connected){
//            if(!timeout){
//                robotinterface_command_position_velocity_acceleration(pva_packet.position,
//                                                                      pva_packet.velocity,
//                                                                      pva_packet.acceleration);
//            }else{ //timeout
//                if(connection_timeout_count<3){
//                    connected=false;
//                    robotinterface_command_velocity(zero_vector);
//                }
//                // cause of timeout we take the last pva_packet
//                robotinterface_command_position_velocity_acceleration(last_pva_packet.position,
//                                                                      last_pva_packet.velocity,
//                                                                      last_pva_packet.acceleration);
//            }
//        }else{
//            robotinterface_command_velocity(zero_vector);
//        }

//        pthread_mutex_unlock(&connect_mutex);

//        robotinterface_send();

//        //------------------------------------------------------------------------------------------------------
//        //------------------------------------------------------------------------------------------------------
//    }

//    // ---- quit programm


    printf("shutdown robot...\n");
    //---- shutdown robot ---------

    // move to home ----
    move_to_position(viereck[0]);
//    move_to_position(joint_home_rad);
//    move_to_position(PosOne);
    // ---
    puts("- Speeding down");
    int j;
    for (j = 0; j < 10; j++)
    {
        robotinterface_read_state_blocking();
        robotinterface_command_velocity(zero_vector);
        robotinterface_send();
    }

    puts("close robotinterface");
    robotinterface_close();
    puts("Done!");

    printf("shutdown program\n");

    return 0;
}

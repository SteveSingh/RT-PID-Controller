#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <mqueue.h>
#include <errno.h>
#include "dlab.h"                        

#define MAXS 5000   // Maximum no of samples
                    // Increase value of MAXS for more samples
pthread_mutex_t mutex_stop = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_modification = PTHREAD_MUTEX_INITIALIZER;
int STOP = 0;
int changed = 0;
sem_t data_avail;   // Do not change the name of this semaphore
                  
float theta[MAXS];  // Array for storing motor position
float ref[MAXS];    // Array for storing reference input
int N = 20;

//Info to be passed to the Control thread
struct thread_info {
  float Fs, run_time, Kp, Ti, Td, Tt, N;
};

//Make declaring thread info types easier
typedef struct thread_info thread_info_t;

float satblk(float v){
  if(v>=-1.4 && v<=1.4)
    v=v;
  
  if (v>=1.4)
    v=1.4;
 
  if (v<=-1.4)
    v=-1.4;
 
  return (v);
}

void *Control(void *arg)
{
  thread_info_t *info;
  
  float Fs, run_time, Kp, T, Ti, Td, N; 
  int samples_taken;
  int k = 0, starting_sample=0;
  float pk, ik, dk, ek, motor_position;
  float ik_prev = 0, ek_prev = 0, dk_prev = 0;//initialize e(k-1), i(k-1) and d(k-1) to 0
  
  //Anti-windup variables
  float Tt = 0.01, at = 0.0, antiwind, v;
  int STOP_INDICATED = 1;
  info = (thread_info_t *)arg;
  Fs = info-> Fs;
  run_time = info-> run_time;
  Kp = info-> Kp;
  Ti = info-> Ti;
  Td = info-> Td;
  Tt = info-> Tt;
  N  = info-> N;
 
  
  T = 1/Fs;  //Period
           
  samples_taken = (int)(run_time * Fs);
      
  
  while(STOP_INDICATED){
       k = starting_sample; //Reset K to iterate samples again from starting_samples
       //this makes sure that we only replace the thetas after 3seconds of sampling IF changed == 1
      pthread_mutex_lock(&mutex_stop); //protect the following data
      if(STOP == 1){
        STOP_INDICATED = 0;
      }    
      pthread_mutex_unlock(&mutex_stop); //protect the following data
      while(k < samples_taken)
      {
        //Acquire semaphore
        sem_wait(&data_avail);

        //Retrieve motor position (radians)
        motor_position = EtoR(ReadEncoder());
        //Determine the error rate                                                
        ek = ref[k] - motor_position;
      
      //Determine the proportional term pk         
        pk = Kp*ek;
      
      if (k == 0){
        ik = 0;
        dk = (Kp * Td*N/(N * T + Td))*(ek);
      }
      else {
       //Determine anti-wind
       antiwind = (at*T)/Tt; 
      //Determine the integral term ik using forward rectangular rule
      //and combine with anti-wind
        ik = ik_prev + (Kp/Ti)*(ek_prev)*T + antiwind;
      //Determine the derivative term dk using backward rectangular rule
      dk = (Td/(N*T + Td))*dk_prev + (((Kp*Td*N)/(N*T+Td))*(ek - ek_prev));
      }

        v = pk + ik + dk;                           //calculate control value
      float uk = satblk(v);
      
      at = uk - v;

        ik_prev = ik;                      //Store current ik into prev variable for future use as i(k-1)
        ek_prev = ek;                      //store current ek into prev variable for future use as e(k-1)
        dk_prev = dk;                      //store current dk into prev variable for future use as d(k-1)

        DtoA(VtoD(uk));                               // It is converted by VtoD and sent to the D/A.
        theta[k] = motor_position;
        k++; 

          pthread_mutex_lock(&mutex_modification); //protect the following data
        if(changed == 1){
          starting_sample = 3.0*Fs;//new starting location before updating values
          k = starting_sample;//start at next samples
          Fs = info->Fs;
         // samples_taken = (int)(run_time * Fs);
          Kp = info->Kp;
          Ti = info->Ti;
          Td = info->Td;
          N = info->N;
          printf("System updated: \n");
          printf("FS %f, Kp %f, Ti %f, Td %f, N %f\n", Fs, Kp, Ti, Td, N);
          changed = 0;
        }
        pthread_mutex_unlock(&mutex_modification); //protect the following data
      }
  }                 
  pthread_exit(NULL); 
}



int main(void *arg)
{
   //Prepare info for the control thread
  thread_info_t thread_info;
  pthread_t control_t;

  //-- Defaults
  float Kp = 21.75;             // Initialize Kp to 1.
  float Ti = 0.025, Td = 0.00625;
  float N = 20;
  float run_time = 3.0; // Set the initial run time to 3 seconds.
  int motor_number = 11;    // Motor number for specific workstation               
  int Fs = 200;             // Sampling frequency (default = 200 Hz)
  
  //-- Plotting variables
  int no_of_points = 50;    // # of points when plotting
  int input_t = 0;          // 0 = step function, 1 = square wave
  
  int i = 0;                // Iterating integer
  int samples_taken;        // # of samples taken
  char choice;              // User choice from menu
  float step_deg = 50, step_rad;//50 mag for step degrees
  float mag = 50, freq = 0.5, dc = 50; //Default magnitude, frequency and duty cycle
  

  while(1)
  {
    //Print out menu
    printf("\n -==== Menu ====- \n");
    printf("\tr: Run the control algorithm continuously.\n");
    printf("\ts: Stop the control algorithm \n");
    printf("\tp: Change value of Kp \n");
    printf("\ti: Change Value of Ti \n");
    printf("\td: Change Value of Td \n");
    printf("\tf: Change the sampling frequency \n");
    printf("\tt: Change the run time \n");
    printf("\tu: Change the input type (Step or Square) \n");
    printf("\tg: Plot results on screen \n");
    printf("\th: Save the Plot results in Postscript \n");
    printf("\tn: Change Value of N \n");
    printf("\tq: exit \n");
    scanf("%s", &choice);
     
     switch(choice){

      //-- Run the control algorithm
      case 'r':
        sem_init(&data_avail, 0, 0);
        if (Initialize(DLAB_SIMULATE, Fs, motor_number) != 0) 
        {
          printf("Error initializing..\n");
          exit(-1);
        }
        
        samples_taken = (int)(run_time * Fs);


        thread_info.Fs = Fs;                    
        thread_info.run_time = run_time;
        thread_info.Kp = Kp;
        thread_info.Ti = Ti;
        thread_info.Td = Td;
        thread_info.N = N;
         thread_info.Tt = 0.01;
         pthread_mutex_lock(&mutex_stop); //protect the following data
          STOP = 0;
         pthread_mutex_unlock(&mutex_stop); //protect the following data
        //Dispatch control thread
        if (pthread_create(&control_t, NULL, &Control, &thread_info) != 0) 
        {
          printf("Error creating Sender thread.\n");
          exit(-1);
        }
        break;                                                                     

    //Stop the continuous run
      case 's':
        pthread_mutex_lock(&mutex_stop); //protect the following data
          STOP = 1;
         pthread_mutex_unlock(&mutex_stop); //protect the following data

       //Wait for control thread to finish
        pthread_join(control_t, NULL);        
        Terminate();
        sem_destroy(&data_avail);
        printf("Simulation stopped!\n");
        break;
      //-- Change value of Kp        
      case 'p':
        printf("Enter new Kp: \n");
        scanf("%f", &Kp);
         pthread_mutex_lock(&mutex_modification); //protect the following data
             thread_info.Kp = Kp;  
             changed = 1;
        pthread_mutex_unlock(&mutex_modification); //protect the following data
        printf("New value of Kp is %f \n", Kp);
      break;

      //Change the value of Ti
      case 'i':
        printf("Enter new Ti: \n");
        scanf("%f", &Ti);
         pthread_mutex_lock(&mutex_modification); //protect the following data
             thread_info.Ti = Ti;  
             changed = 1;
        pthread_mutex_unlock(&mutex_modification); //protect the following data
        printf("New value of Ti is %f \n", Ti);
      break;

      //Change the value of Td
      case 'd':
        printf("Enter new Td: \n");
        scanf("%f", &Td);
         pthread_mutex_lock(&mutex_modification); //protect the following data
             thread_info.Td = Td;  
             changed = 1;
       pthread_mutex_unlock(&mutex_modification); //protect the following data
        printf("New value of Td is %f \n", Td);
      break;
        
      //Change the value of N
      case 'n':
        printf("Enter new N: \n");
        scanf("%f", &N);
         pthread_mutex_lock(&mutex_modification); //protect the following data
             thread_info.N = N;  
             changed = 1;
        pthread_mutex_unlock(&mutex_modification); //protect the following data
        printf("New value of N is %f \n", N);
      break;

      //-- Change value of sampling_freq
      case 'f':
        printf("Enter new sampling_freq: \n");
        scanf("%f", &Fs);
        pthread_mutex_lock(&mutex_modification); //protect the following data
             thread_info.Fs = Fs;  
             changed = 1;
        pthread_mutex_unlock(&mutex_modification); //protect the following data
        printf("New sampling frequency is %f \n", Fs);
      break;

      //-- Change value of run_time      
      case 't':
        printf("Enter new run_time: \n");
        scanf("%f", &run_time);
        printf("New run-time is %f \n", run_time);
      break;

      //-- Change the type of inputs      
      case 'u':
        printf("Select input type (0 = step | 1 = square): \n");
        scanf("%d", &input_t);
        if (input_t == 0)
        {   //Step Input
          printf("Enter degrees: \n");
          scanf("%f", &step_deg);
          step_rad = step_deg * (PI/180);
          printf("Step value (radians) = %f\n", step_rad);
          
          //Populate reference array with step input
          for (i = 0; i < MAXS; i++)
          {
            ref[i] = step_rad;
          }
          
        }
        else if (input_t == 1) 
        { 
          //Square Input
          printf("Enter magnitude: \n");
          scanf("%f", &mag);
          printf("Enter Frequency: \n");
          scanf("%f", &freq);
          printf("Enter duty cycle: \n");
          scanf("%f", &dc);

          //Populate reference array with square wave input
          Square(ref, MAXS, Fs, mag*(PI/180.0), freq, dc);
        }
      
      break;

      //-- Plot results on screen        
      case 'g':
        // Plot the graph of reference and output vs time on the screen
        no_of_points = run_time * Fs;
        plot(ref, theta, Fs, no_of_points, SCREEN, "PID Response", "t (s)", "Motor Position");

      break;

      //-- Save the Plot results in Postscript        
      case 'h':
        // Save the graph of reference and output vs time in Postscript
        no_of_points = run_time * Fs;
        plot(ref, theta, Fs, no_of_points, PS, "PID Response", "t (s)", "Motor Position");
      break;

      //-- Exit      
      case 'q':
        printf("Finished.");
        pthread_exit(NULL);
      break; 
      
      default:
        printf("Invalid choice.\n");
      break;
     }
    
  }
}

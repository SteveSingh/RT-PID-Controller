/*
  Authors: 
  Steve Singh   | 500389934
  Rishabh Kumar | 500398457
*/
  
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

int data_avail;   // Do not change the name of this semaphore
                  
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

  // Declare an RTAI RT task
unsigned long control_task_name;
RT_TASK *control_task;

  thread_info_t *info;
  
  float Fs, run_time, Kp, T, Ti, Td, N; 
  int samples_taken;
  int k = 0;
  float pk, ik, dk, ek, motor_position;
  float ik_prev = 0, ek_prev = 0, dk_prev = 0;//initialize e(k-1), i(k-1) and d(k-1) to 0
  
  //Anti-windup variables
  float Tt = 0.01, at = 0.0, antiwind, v;

    // Allow non-root user to use POSIX soft real-time process management
    // and memory lock functions.
    rt_allow_nonroot_hrt();
    // Register this thread with RTAI
    control_task_name = nam2num("CONTROL");
    control_task = rt_task_init(control_task_name, 0, 0, 0);
    // Make this a hard real-time task
    rt_make_hard_real_time();
    // Prevent paging by locking the task in memory
    mlockall(MCL_CURRENT | MCL_FUTURE);


  info = (thread_info_t *)arg;
  Fs = info-> Fs;
  run_time = info-> run_time;
  Kp = info-> Kp;
  Ti = info-> Ti;
  Td = info-> Td;
  Tt = info-> Tt;
  N  = info-> N;
 
	
	T = 1/Fs;	 //Period
           
  samples_taken = (int)(run_time * Fs);
      
  
  
  while(k < samples_taken)
  {
    //Acquire semaphore
    rtf_sem_wait(data_avail);


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
  }                   


  // reset to soft real-time
rt_make_soft_real_time();
// un-register the control thread from RTAI
rt_task_delete(control_task);
// re-enable paging
munlockall();

  pthread_exit(NULL); 
}



int main(void *arg)
{
  pthread_t control_t;

  //-- Defaults
  float Kp = 21.75;             // Initialize Kp to 1.
  float Ti = 0.025, Td = 0.00625;
  float N = 20;
  float run_time = 20.0;     // Set the initial run time to 3 seconds.
  int motor_number = 11;    // Motor number for specific workstation               
  int Fs = 200;             // Sampling frequency (default = 200 Hz)
  
  //-- Plotting variables
  int no_of_points = 50;    // # of points when plotting
  int input_t = 0;          // 0 = step function, 1 = square wave
  
  int i = 0;                // Iterating integer
  int samples_taken;        // # of samples taken
  char choice;              // User choice from menu
  float step_deg, step_rad;
  float mag, freq, dc;
  

  while(1)
  {
    //Print out menu
    printf("\n\t -==== Menu ====- \n");
    printf("\tr: Run the control algorithm \n");
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
        if ( (data_avail = open("/dev/rtf0", O_WRONLY)) < 0 ) {
            printf("Error: cannot open /dev/rtf0.\n");
        exit(1);
        }
        if ( rtf_sem_init(data_avail, 0) != 0) {
           printf("Error: cannot init RT FIFO semaphore.\n");
        }
        if (Initialize(DLAB_HARDWARE, Fs, motor_number) != 0) 
        {
          printf("Error initializing..\n");
          exit(-1);
        }
        
        samples_taken = (int)(run_time * Fs);

        //Prepare info for the control thread
        thread_info_t thread_info;
        thread_info.Fs = Fs;                    
        thread_info.run_time = run_time;
        thread_info.Kp = Kp;
        thread_info.Ti = Ti;
        thread_info.Td = Td;
        thread_info.N = N;
		    thread_info.Tt = 0.01;
     
        int period_in_ticks;
        period_in_ticks = (int)( ((double) 1.0e9)/((double) Fs));
        start_rt_timer(period_in_ticks);

        //Dispatch control thread
        if (pthread_create(&control_t, NULL, &Control, &thread_info) != 0) 
        {
          printf("Error creating Sender thread.\n");
          exit(-1);
        }
        
        //Wait for control thread to finish
        pthread_join(control_t, NULL);
        //Terminate motor connection
        Terminate();
        //Destroy semaphore
        rtf_sem_destroy(data_avail);
        close(data_avail);
        stop_rt_timer(); 
      break;                                                                     

      //-- Change value of Kp        
      case 'p':
        printf("Enter new Kp: \n");
        scanf("%f", &Kp);
        printf("New value of Kp is %f \n", Kp);
      break;

      //Change the value of Ti
      case 'i':
        printf("Enter new Ti: \n");
        scanf("%f", &Ti);
        printf("New value of Ti is %f \n", Ti);
      break;

      //Change the value of Td
      case 'd':
        printf("Enter new Td: \n");
        scanf("%f", &Td);
        printf("New value of Td is %f \n", Td);
      break;
        
      //Change the value of N
      case 'n':
        printf("Enter new N: \n");
        scanf("%f", &N);
        printf("New value of N is %f \n", N);
      break;

      //-- Change value of sampling_freq
      case 'f':
        printf("Enter new sampling_freq: \n");
        scanf("%f", &Fs);
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
        plot(ref, theta, Fs, no_of_points, SCREEN, "PID Step Response", "t (s)", "Motor Position");

      break;

      //-- Save the Plot results in Postscript        
      case 'h':
        // Save the graph of reference and output vs time in Postscript
        no_of_points = run_time * Fs;
        plot(ref, theta, Fs, no_of_points, PS, "PID Step Response", "t (s)", "Motor Position");
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

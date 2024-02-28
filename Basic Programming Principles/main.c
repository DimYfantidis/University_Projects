#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//Metric functions
int frequency(char *filename,int n) {
  FILE *fptr;
  int i,a;
  int fr=0;

  fptr = fopen(filename,"r");
  if (fptr!=NULL) {
    while (!feof(fptr)) {
      fscanf(fptr,"%d",&a);
      if (n==a)
        fr++;
    }
    fclose(fptr);
  }
  else {
    printf("Error opening file %s\n",filename);
    return -1;
  }

  return fr;
}

int delay(char *filename,int n) {
  FILE *fptr;
  int i,a=-1;
  int del = 0;
  
  fptr = fopen(filename,"r");
  if (fptr!=NULL) {
    while (!feof(fptr) && a!=n) {
      for (i=0;i<7;i++) {
        fscanf(fptr,"%d",&a);
        if (a==n)
          break;
      }
      del++;
    }
    fclose(fptr);
  }
  else {
    printf("Error opening file %s\n",filename);
    return -1;
  }
  
  if (a==n)
    return del-1;
  else
    return del;
}

int frPlusDel(char *filename,int n) {
  return frequency(filename,n) + delay(filename,n);
}

float relativeDelay(char *filename,int n,int K) {
  int F = frequency(filename,n);
  if (F!=0)
    return delay(filename,n) - (float)K/F;
  else
    return 0;
}

//Auxiliary Functions
void distinctElements(float *A,int *size) {
  int i,j,k;

  for (i=0;i<*size;i++) {
    for (j=i+1;j<*size;j++) {
      if (A[i]==A[j]) {
        for (k=j+1;k<*size;k++)
          A[k-1] = A[k];
        j--;
        (*size)--;
      }
    }
  }
  return;
}

void bubbleSort(float *A,int size) {
  int i, swap;
  float temp;
  while (1) {
    swap = 0;
    for (i=0;i<size-1;i++) {
      if (A[i]>A[i+1]) {
        temp = A[i];
        A[i] = A[i+1];
        A[i+1] = temp;
        swap = 1;
      }
    }
    if (swap==0)
      break;
  }
  return;
}

void printArray(float *A,int size){
  int i;
  printf("\n");
  for (i=0;i<size;i++) {
    printf("%2.2f ",A[i]);
    if ((i+1)%7==0)
      printf("\n");
  }
  printf("\n");
}

//Main Program
int main()
{
  FILE *fp;
  int numbers, metric;
  char *filename;

  int i,j,k;
  int K=0;

  scanf("%d %d ",&numbers,&metric);

  //String of file to be opened
  filename = malloc(1000*sizeof(char));
  fgets(filename,1000,stdin);
  filename[strcspn(filename,"\n")] = '\0';
  filename = realloc(filename,(strlen(filename)+1)*sizeof(char));
  
  //Value check
  if (numbers <= 0 || numbers >=50 || metric<=-1 || metric>=4) {
    printf("Wrong Input!\n");
    exit (1);
  }
  //File check
  fp = fopen(filename,"r");
  if (fp==NULL) {
    printf("File Error!\n");
    exit (2);
  }

  int *N;
  int total=0,count=0;

  N = malloc(sizeof(int));

  while (!feof(fp)) {
    fscanf(fp,"%d ",&N[total]);
    total++;
    N = realloc(N,(total+1)*sizeof(int));
    if (total%7==0)
      K++;
  }
  fclose(fp);

  distinctElements((float *)N,&total);
  bubbleSort((float *)N,total);

  //Metric 0
  if (metric==0) {
    int fr_total = total;
    int *frequency_arr;
    frequency_arr = calloc(total,sizeof(int));

    for (i=0;i<total;i++) 
      frequency_arr[i] = frequency(filename,N[i]);

    distinctElements((float *)frequency_arr,&fr_total);
    bubbleSort((float *)frequency_arr,fr_total);

    k = 0;
    while (k<numbers) {
      for (i=fr_total-1;i>=0 && k<numbers;i--) {
        for (j=0;j<total && k<numbers;j++) {
          if (frequency(filename, N[j])==frequency_arr[i]) {
            printf("%d ",N[j]);
            k++;
          }
        }
      }
    }
  }

  //Metric 1
  else if (metric==1) {
    int del_total = 49;
    int *delay_arr;
    delay_arr = calloc(del_total,sizeof(int));

    for (i=0;i<del_total;i++)
      delay_arr[i] = delay(filename,i+1);

    distinctElements((float *)delay_arr,&del_total);
    bubbleSort((float *)delay_arr,del_total);

    k=0;
    while (k<numbers) {
      for (i=del_total-1;i>=0 && k<numbers;i--) {
        for (j=0;j<49 && k<numbers;j++) {
          if (delay(filename,j+1)==delay_arr[i]) {
            printf("%d ",j+1);
            k++;
          }
        }
      }
    }
  }

  //Metric 2
  else if (metric==2) {
    int fd_total = 49;
    int *fd_arr;
    fd_arr = calloc(fd_total,sizeof(int));

    for (i=0;i<fd_total;i++)
      fd_arr[i] = frPlusDel(filename,i+1);

    distinctElements((float *)fd_arr,&fd_total);
    bubbleSort((float *)fd_arr,fd_total);

    k=0;
    while (k<numbers) {
      for (i=fd_total-1;i>=0 && k<numbers;i--) {
        for (j=0;j<49 && k<numbers;j++) {
          if (frPlusDel(filename,j+1)==fd_arr[i]) {
            printf("%d ",j+1);
            k++;
          }
        }
      }
    }
  }

  //Metric 3
  else {
    int rd_total = 49;
    float *rdel_arr;
    rdel_arr = calloc(rd_total,sizeof(float));

    for (i=0;i<rd_total;i++)
      rdel_arr[i] = relativeDelay(filename,i+1,K);

    distinctElements(rdel_arr,&rd_total);
    bubbleSort(rdel_arr,rd_total);

    k=0;
    while (k<numbers) {
      for (i=rd_total-1;i>=0 && k<numbers;i--) {
        for (j=0;j<49 && k<numbers;j++) {
          if (relativeDelay(filename,j+1,K)==rdel_arr[i]) {
            printf("%d ",j+1);
            k++;
          }
        }
      }
    }
  }

  return 0;
}
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include <semaphore.h>

typedef struct {
    int             count;
    pid_t           pid;
} ST_MUTEX_MEM;


class DEVICE_MUTEX
{
public:
    DEVICE_MUTEX( const char* shm_key, int timeout_sec=1 )
    {
        if( (shm_id = shmget( ftok( shm_key, 'R' ), 80, IPC_CREAT|IPC_EXCL|0666)) == -1){
            if( errno == EEXIST ){
                perror("exist");
                shm_id = shmget( ftok( shm_key, 'R' ), 80, 0666);
                perror("exis");
            }else{
                perror("shmget");
                exit(-1);
            }
        }
        mutex_mem = (ST_MUTEX_MEM *)shmat( shm_id, NULL, 0 );
        // printf("debug %d %08x %d\r\n",shm_id,(void*)mutex_mem, mutex_mem->pid );
        if( mutex_mem == (void *)-1 ){
            perror("shmat");
        }

        int fa;
        for( fa=strlen(shm_key)-1 ; fa > 0 ; --fa ){
            if( shm_key[fa] == '/' ){
                break;
            }
        }

        sem = sem_open(&shm_key[fa], O_CREAT, (S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH), 1); 
        lock_flg = false;
        if( sem == SEM_FAILED ){
            perror("semopen");
        }

        timespec timeout;
        clock_gettime( CLOCK_REALTIME, &timeout);
        timeout.tv_sec += 1;
        if( (sem_timedwait( sem, &timeout ) != 0) && (errno == ETIMEDOUT) ){
            if( !is_alive(mutex_mem->pid) ){
                int semval;
                sem_getvalue( sem, &semval );
                if( semval <= 0 ){
                    sem_post( sem );
                }
                mutex_mem->pid = getpid();
                mutex_mem->count = 0;
            }
        }else{
            mutex_mem->pid = getpid();
            unlock();
        }

        ++mutex_mem->count;
    }
    virtual ~DEVICE_MUTEX()
    {
        --mutex_mem->count;
        if( lock_flg ){
            unlock();
        }
        sem_close( sem );
        shmdt( mutex_mem );
        shmctl(shm_id, IPC_RMID, 0);
    }
    void lock()
    {
        sem_wait( sem );
        mutex_mem->pid = getpid();
        lock_flg = true;
    }
    void unlock()
    {
        sem_post( sem );
        mutex_mem->pid = 0;
        lock_flg = false;
    }
private:
    bool is_alive( pid_t check_pid )
    {
	    char filepath[256];
        struct stat get_stat;
        snprintf( filepath, sizeof(filepath), "/proc/%d", (int)check_pid );
        return (stat(filepath,&get_stat)==0)?true:false;
    }
    int shm_id;
    int timeout;
    ST_MUTEX_MEM* mutex_mem;
    sem_t* sem;
    bool lock_flg;
};




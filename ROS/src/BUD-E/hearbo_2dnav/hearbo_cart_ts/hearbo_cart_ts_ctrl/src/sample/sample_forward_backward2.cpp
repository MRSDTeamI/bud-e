/*
 * iclient.c
 *    �����åȤ���Ѥ��ơ������С�����³���륯�饤����ȥץ���ࡣ
 *    
 *    ���Ϥ��줿ʸ����򥵡��С������ꡢ�����С�����ʸ�����Ѵ������ǡ�����
 *    ������롣
 * 
 */

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <iostream>
#include <string>
#include <string.h>
#include <sys/time.h>

#define PORT 10001

using namespace std;

main(int argc, char *argv[])
{
    struct sockaddr_in    addr;
    struct hostent *hp;
    int    fd;
    int    len;
    char   buf[1024];
    int    ret;

    unsigned char   bufuc[1024];

    if (argc != 2){
	printf("Usage: iclient SERVER_NAME\n");
    }

    /*
     *  �����åȤ��롣���Υ����åȤ�UNIX�ɥᥤ��ǡ����ȥ꡼�෿�����åȡ�
     */
    if ((fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket");
    }

    /* 
     * addr����Ȥ�0�ˤ��Ƥ����ʤ��ȡ�bind()�ǥ��顼�������뤳�Ȥ�����
     */
    bzero((char *)&addr, sizeof(addr));

    /*
     * �����åȤ�̾��������Ƥ���
     */


    if ((hp = gethostbyname(argv[1])) == NULL) {
	perror("No such host");
    }
    bcopy(hp->h_addr, &addr.sin_addr, hp->h_length);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);

    /*
     *  �����С��Ȥ���³���ߤ롣���줬�������뤿��ˤϡ������С������Ǥ�
     *  ���Υ��ɥ쥹��bind()���ơ�listen()��ȯ�Ԥ��Ƥ��ʤ���Фʤ�ʤ���
     */
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0){
        perror("connect");
    }
    /*
     *  ���Ϥ��줿�ǡ����򥽥��åȤ˽񤭹���ǥ����С������ꡢ
     *  �����С����Ѵ��������äƤ����ǡ������ɤ߹��ࡣ
     */

    string buftmp;

    /*----------------------------------------------------------------*/
    /* Main routine (ex. small_rotation )*/

    char pcCommand[257];
    char pcCommandBuffer[520];

    fgets(buf, 1024, stdin);
    sprintf( pcCommand, "%s", "#b1" );
    sprintf(pcCommandBuffer, "%s\r", pcCommand ); // Append carriage return    
    send(fd, pcCommandBuffer, strlen( pcCommandBuffer ), 0);


    fgets(buf, 1024, stdin);
    sprintf(pcCommand, "#bm%+04d%+04d000", 10, 0);
    // sprintf(pcCommand, "%s", "#bm+010+000000");
    sprintf(pcCommandBuffer, "%s\r", pcCommand ); // Append carriage return    
    send(fd, pcCommandBuffer, strlen( pcCommandBuffer ), 0);

    fgets(buf, 1024, stdin);
    sprintf(pcCommand, "#bm%+04d%+04d000", -10, 0);
    // sprintf(pcCommand, "%s", "#bm-010+000000");
    sprintf(pcCommandBuffer, "%s\r", pcCommand ); // Append carriage return    
    send(fd, pcCommandBuffer, strlen( pcCommandBuffer ), 0);

    fgets(buf, 1024, stdin);
    sprintf( pcCommand, "%s", "#bu" );
    sprintf(pcCommandBuffer, "%s\r", pcCommand ); // Append carriage return    
    send(fd, pcCommandBuffer, strlen( pcCommandBuffer ), 0);

    close(fd);
}









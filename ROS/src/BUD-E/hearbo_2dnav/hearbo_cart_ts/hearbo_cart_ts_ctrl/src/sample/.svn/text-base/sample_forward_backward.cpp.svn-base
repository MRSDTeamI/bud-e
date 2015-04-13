/*
 * iclient.c
 *    ソケットを使用して、サーバーに接続するクライアントプログラム。
 *    
 *    入力された文字列をサーバーに送り、サーバーが大文字に変換したデータを
 *    受け取る。
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
     *  ソケットを作る。このソケットはUNIXドメインで、ストリーム型ソケット。
     */
    if ((fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket");
    }

    /* 
     * addrの中身を0にしておかないと、bind()でエラーが起こることがある
     */
    bzero((char *)&addr, sizeof(addr));

    /*
     * ソケットの名前を入れておく
     */


    if ((hp = gethostbyname(argv[1])) == NULL) {
	perror("No such host");
    }
    bcopy(hp->h_addr, &addr.sin_addr, hp->h_length);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);

    /*
     *  サーバーとの接続を試みる。これが成功するためには、サーバーがすでに
     *  このアドレスをbind()して、listen()を発行していなければならない。
     */
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0){
        perror("connect");
    }
    /*
     *  入力されたデータをソケットに書き込んでサーバーに送り、
     *  サーバーが変換して送ってきたデータを読み込む。
     */

    string buftmp;

    /*----------------------------------------------------------------*/
    /* Main routine (ex. small_rotation )*/

    char pcCommand[257];
    char pcCommandBuffer[520];

    fgets(buf, 1024, stdin);
    buftmp = "";
    buftmp = "#b1\r";
    write(fd, buftmp.c_str(), sizeof(unsigned char) * buftmp.size() );

    fgets(buf, 1024, stdin);
    buftmp = "";
    buftmp = "#bm+010+000000\r";
    write(fd, buftmp.c_str(), sizeof(unsigned char) * buftmp.size() );

    fgets(buf, 1024, stdin);
    buftmp = "";
    buftmp = "#bm-010+000000\r";
    write(fd, buftmp.c_str(), sizeof(unsigned char) * buftmp.size() );

    fgets(buf, 1024, stdin);
    buftmp = "";
    buftmp = "#bu\r";
    write(fd, buftmp.c_str(), sizeof(unsigned char) * buftmp.size() );

    close(fd);
}









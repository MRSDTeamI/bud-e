#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <~/ffmpeg_sources/ffmpeg/libavformat/avformat.h>
#include <libavcodec/avcodec.h>

#include <time.h>
#include <libhtk3.h>
#include <libhtkIO/err.h>
#include <libhtkIO/ad.h>

//currently not in use
/*int findWord(ps_decoder_t *ps, int32_t score, char *word) {
    ps_seg_t *seg = ps_seg_iter(ps, &score);
    char const *currentWord;
    int totalFrames = ps_get_n_frames(ps);
    int allTimes[totalFrames]; //array that holds all times this word occurs
    int num = 0; 
    
    ps_start_stream(ps);

    while(seg != NULL){
        int32_t sf, ef;
        
        ps_seg_frames(seg, &sf, &ef);
        currentWord = ps_seg_word(seg);
        //if we found the word you're looking for
        if(strcmp(word, ps_seg_word(seg)) == 0){
            //add the start frame for this word to the list of all times
            allTimes[num] = sf;
            num++;
        }
        seg = ps_seg_next(seg); //go to next word in segmentation
    }
    if(allTimes == NULL)
        return -1;
    else
        return allTimes[0];
}*/

static ps_decoder_t *ps;
static cmd_ln_t *config;
static FILE* rawfd;
static int findWord;
static const char *find;

static int32 ad_file_read(ad_rec_t *ad, int16 *buf, int32 max) {
    size_t nread;

    nread = fread(buf, sizeof(int16), max, rawfd);

    return (nread > 0 ? nread : -1);
}

static int print_word_times() {
    char const *currentWord;
    int frame_rate = cmd_ln_int32_r(config, "-frate");
    ps_seg_t *seg = ps_seg_iter(ps, NULL);
    
    while(seg != NULL) {
        int32 sf, ef, pprob;
        float conf;

        ps_seg_frames(seg, &sf, &ef);
        currentWord = ps_seg_word(seg);
        //if we found the word you're looking for
        if((findWord == 1) && strcmp(find, currentWord) == 0){
            printf("FOUND: %s at %.3f seconds\n", find, ((float)sf / frame_rate));
            return 1;
        }
        pprob = ps_seg_prob(seg, NULL, NULL, NULL);
        conf = logmath_exp(ps_get_logmath(ps), pprob);
        printf("%s %.3f %.3f %f\n", ps_seg_word(seg), ((float)sf / frame_rate), ((float)ef / frame_rate), conf);
        seg = ps_seg_next(seg);
    }
    return 0;
}


static int check_wav_header(char *header, int expected_sr) {
    int sr;

    if(header[34] != 0x10) {
        E_ERROR("Input audio file has [%d] bits per sample instead of 16\n", header[34]);
        return 0;
    }
    if(header[20] != 0x1) {
        E_ERROR("Input audio file has compression [%d] and not required PCM\n", header[20]);
        return 0;
    }
    if(header[22] != 0x1) {
        E_ERROR("Input audio file has [%d] channels, expected single channel mono\n", header[22]);
        return 0;
    }
    sr = ((header[24] & 0xFF) | ((header[25] & 0xFF) << 8) | ((header[26] & 0xFF) << 16) | ((header[27] & 0xFF) << 24));
    if(sr != expected_sr) {
        E_ERROR("Input audio file has sample rate [%d], but decoder expects [%d]\n", sr, expected_sr);
        return 0;
    }
    return 1;
}


static void recognize_from_file(const char *fname) {
    int16 adbuf[2048];
    //const char *fname;
    const char *hyp;
    int32 k;
    uint8 utt_started, in_speech;

    //fname = cmd_ln_str_r(config, "-infile");
    //fname = "colon2clean.wav";
    if ((rawfd = fopen(fname, "rb")) == NULL) {
        E_FATAL_SYSTEM("Failed to open file '%s' for reading", fname);
    }
    
    if(strlen(fname) > 4 && strcmp(fname + strlen(fname) -4, ".wav") == 0) {
        char waveheader[44];
        fread(waveheader, 1, 44, rawfd);
        if(!check_wav_header(waveheader, (int)cmd_ln_float32_r(config, "-samprate")))
            E_FATAL("Failed to process file '%s' due to format mismatch.\n", fname);
    }

    ps_start_utt(ps);
    utt_started = FALSE;

    while((k = fread(adbuf, sizeof(int16), 2048, rawfd)) > 0) {
        ps_process_raw(ps, adbuf, k, FALSE, FALSE);
        in_speech = ps_get_in_speech(ps);
        if(in_speech && !utt_started) {
            utt_started = TRUE;
        }
        if(!in_speech && utt_started) {
            ps_end_utt(ps);
            hyp = ps_get_hyp(ps, NULL);
            printf("%s\n", hyp);
            print_word_times();

            ps_start_utt(ps);
            utt_started = FALSE;
        }
    }
    ps_end_utt(ps);
    if(utt_started) {
        hyp = ps_get_hyp(ps, NULL);
        printf("%s\n", hyp);
        print_word_times();
    }
    fclose(rawfd);
}


int main (int argc, char *argv[]) {
    //ps_decoder_t *ps;
    //ps_seg_t *seg; //segmentation iterator object
    //cmd_ln_t *config;
    FILE *fh;
    char const *hyp, *uttid;
    int rv;
    int32_t score;
    
    printf("Creating configuration object.....\n");

    //create a configuration object 
    //cmd_ln_init takes a variable number of null-terminated string args
    //first arg is any previous cmd_ln_t which is to be updated
    //second is an array of argument definitions (std set obtained by calling
    //ps_args())
    //3rd is a flag telling the arg parser to be strict
    //MODELDIR macro defined on the GCC command line
    
    config = cmd_ln_init(NULL, ps_args(), TRUE, 
                        "-hmm", MODELDIR "/en-us/en-us",
                        "-lm", MODELDIR "/en-us/bude.lm",
                        "-dict", MODELDIR "/en-us/bude.dict",
                        "-frate", "100",
                        NULL);
    if(config == NULL){
        printf("Configuration failed\n");
        return 1;
    }

    //initialize the decoder
    ps = ps_init(config);
    if(ps == NULL){
        cmd_ln_free_r(config);
        return 1;
    }

    //"rb" is the mode of fopen "rb" signifies that not text (versus just r)
    fh = fopen("input.rosbag.wav", "rb");
    //make sure it opened
    if(fh == NULL) {
        perror("Failed to open file");
        return 1;
    }

    printf("Opened file, about to decode\n");

    //decode the file
    //ps_decode_raw takes a decoder (ps), file (fh), and max number 
    //of samples to read from (-1 to read til end of file)
    /*rv = ps_decode_raw(ps, fh, -1);
    if (rv < 0)
        return 1;
    printf("Finished decoding\n");
    //get the hypothesis
    //ps_get_hyp: get hypothesis string and path score
    //returns string containing best hypothesis at this point in decoding
    //takes: ps_decoder (ps), out_best_score (int32*)
    hyp = ps_get_hyp(ps, &score);
    if (hyp == NULL)
        return 1;
    printf("Recognized: %s\n", hyp);
    
    printWordTimes(ps, score);
    printf("total frames search: %d\n", ps_get_n_frames(ps));

    printf("time:%d\n", findWord(ps,score,"Buddy"));
*/
   

    //parse user input to determine functionality
    findWord = 0;
    switch(argc){
        case 1:
            printf("Please send the audio stream to the node for decoding\n");
            break;
        case 3: 
            findWord = 1;
            find = argv[2];
        case 2:
            recognize_from_file(argv[1]);
            break;
        default:
            printf("too many arguments\n");
            break;
    }
            
    fclose(fh);
    ps_free(ps);
    cmd_ln_free_r(config);
    return 0;

}

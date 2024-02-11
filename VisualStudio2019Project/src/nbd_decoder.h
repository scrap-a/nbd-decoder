struct CHUNK
{
	long chunk_identifier;
	long chunk_size;
};

struct RIFF_HEADER
{
	long  riff_identifier;
	long  file_size;
	long  format;
	long  fmt_identifier;
	long  fmt_size;
	short fmt;
	short ch_num;
	long  sample_rate;
	long  byte_rate;
	short block_size;
	short bit_depth;
	// Extended Format Not Supported

	struct CHUNK data_chunk;
};

typedef enum
{
	AM_MONO_HIGH,
	AM_MONO_LOW,
	AM_STEREO_HIGH,
	AM_STEREO_LOW,
}READ_MODE;

#define DEF_MODE AM_MONO_HIGH
#define SAMPLE_MAX (32767)
#define SAMPLE_MIN (-32768)
#define RIFF_MARK 0x46464952	//"RIFF"
#define FMT_WAVE 0x45564157		//"WAVE"
#define AFMT_LPCM 0x0001		//Linear PCM
#define BDEPTH_16 0x0010		//16bit
#define DATA_MARK 0x61746164	//"data"

#define BIOS_SIZE 0x80000
#define BIOS_SIZE_SHORT (BIOS_SIZE/2)
#define alpha  1.0


int read_wavheader(struct RIFF_HEADER* header, FILE* fin);
int compare(const void* a, const void* b);
short median(short* buf, int size);
short maximum(short* buf, int size, int base);
int am_decode(struct RIFF_HEADER* header, int quant_bit, FILE* fin, FILE* fout);
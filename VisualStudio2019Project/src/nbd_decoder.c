#pragma once
#pragma warning(disable: 4996)

#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include "nbd_decoder.h"

int read_wavheader(struct RIFF_HEADER* header, FILE* fin) {
	int ret = 0;
	int size;

	//fseek(fin, 0, SEEK_SET);
	fread(header, 1, sizeof(*header) - sizeof(header->data_chunk), fin);
	
	if (header->fmt_size != 0x10) {
		fseek(fin, header->fmt_size - 0x10, SEEK_CUR);
	}
	while (header->data_chunk.chunk_identifier != DATA_MARK) {
		size = fread(&header->data_chunk, 1, sizeof(header->data_chunk), fin);
		if (header->data_chunk.chunk_identifier != DATA_MARK) {
			fseek(fin, header->data_chunk.chunk_size, SEEK_CUR);
		}
		if (size == 0) {
			ret = -1;
			break;
		}
	}
	return ret;
}

int compare(const void* a, const void* b) {
	return *((short*)a) - *((short*)b);
}

short median(short* buf, int size) {
	short ret = 0;
	short tmp[64];

	memcpy(tmp, buf, sizeof(short) * size);
	qsort(tmp, size, sizeof(short), compare);

	ret = tmp[(int)(size / 2)];

	return ret;
}

short maximum(short* buf, int size, int base) {
	short ret = 0;
	int i;

	for (i = 0; i < size; i++) {
		if (abs(ret) < abs(buf[i] - base)) {
			ret = buf[i] - base;
		}
	}

	return ret;
}

int am_decode(struct RIFF_HEADER* header, int quant_bit, FILE* fin, FILE* fout) {
	int ret = 0;
	int i;
	int data_cnt = 0;			//推定中のサンプル数
	int dif = 0;				//前のサンプルとの差分値（値が大きい場合に振幅が変化したと判定）
	int med[16];				//推定中のサンプルの中央値を保存
	int med_cnt = 0;			//中央値バッファの現在位置
	int med_lv[16];				//振幅が+か-かbaseか
	int search_step = 0;		//0:search pilot signal / 1:decode
	int amp_limit = (int)(SAMPLE_MAX * 0.8);	//PCMのサンプル推定最大値
	int base = 0;				//振幅の初期値

	const float th0 = (float)0.256;	// (1552+1552/8) / 2048 * 0.3
	const float th1 = (float)0.379;	// (1552)*0.5 / 2048
	const float th2 = (float)0.947;	// (1552+1552/8 +1552/8) / 2048
	const float th3 = (float)1.137;	// (1552+1552/4+1552/8 +1552/8) / 2048
	const float th4 = (float)1.326;	// (1552+1552/2+1552/8 +1552/8) / 2048

	short out = 0;				//出力値(2Byte分)
	int out_cnt = 0;			//出力のbit数
	short* out_buf;				//全出力値のバッファ
	short* in_buf;				//ch毎の入力値バッファ
	short* in_buf2;				//全chの入力値バッファ
	int bps_cnt = 0;			//bpsカウント(デバッグ)
	int last_detect = 0x7FFFFFFF;	//最後にデータ出力が確定したサンプル
	int base_flag = 0;

	out_buf = malloc(sizeof(short) * BIOS_SIZE_SHORT);
	memset(out_buf, 0, BIOS_SIZE_SHORT);
	memset(med, 0, sizeof(int) * 16);
	memset(med_lv, 0, sizeof(int) * 16);

	in_buf = (short*)malloc(sizeof(short) * header->data_chunk.chunk_size / header->ch_num / 2);
	in_buf2 = (short*)malloc(sizeof(short) * header->data_chunk.chunk_size / 2);
	fread(in_buf2, sizeof(short), header->data_chunk.chunk_size / 2, fin);
	for (int k = 0; k < header->data_chunk.chunk_size / header->ch_num / 2; k++) {
		in_buf[k] = in_buf2[header->ch_num * k];
	}
	free(in_buf2);

	for (i = 0; i < header->data_chunk.chunk_size / header->ch_num / 2; i++) {

		//最後に検出した箇所から3データ分何も検出されなければ、pilot_signal検出へ移行
		if (search_step == 1 && (i - last_detect) > (int)(header->sample_rate * 3 / 18500) + 1) {
			search_step = 0;
			med_cnt = 0;
			memset(med, 0, sizeof(int) * 16);
			memset(med_lv, 0, sizeof(int) * 16);
		}

		//差分値算出
		if (data_cnt >= 1)
			dif = in_buf[i] - in_buf[i-1];
		else dif = 0;

		data_cnt++;
		data_cnt = data_cnt % 64;

		if (quant_bit == 2) {

			//差分値が、振幅の取り得る値の最低値/3よりも大きい場合(pilot_signal検出モード)
			if (abs(dif) > amp_limit * th0 && search_step == 0) {
				med[med_cnt] = median(&in_buf[i - data_cnt + 1], data_cnt == 0 ? 1 : data_cnt - 1);
				//med[med_cnt] = in_buf[i - (int)(data_cnt / 2.0)];

				if (med[med_cnt] > amp_limit * th0) {
					med_lv[med_cnt] = 1;
				}
				else if (med[med_cnt] < -amp_limit * th0) {
					med_lv[med_cnt] = -1;
				}
				else {
					med_lv[med_cnt] = 0;
				}

				//check pilot signal
				if (med_lv[med_cnt] == 0 &&
					med_lv[(med_cnt + 15) % 16] == -1 &&
					med_lv[(med_cnt + 14) % 16] == 0 &&
					med_lv[(med_cnt + 13) % 16] == -1 &&
					med_lv[(med_cnt + 12) % 16] == 0 &&
					med_lv[(med_cnt + 11) % 16] == 1 &&
					med_lv[(med_cnt + 10) % 16] == 0 &&
					med_lv[(med_cnt + 9) % 16] == 1 &&
					med_lv[(med_cnt + 8) % 16] == 0
					) {
					search_step = 1;
					amp_limit = med[(med_cnt + 11) % 16] * SAMPLE_MAX / 31504;
					//base = med[(med_cnt + 14) % 16];
					base = med[(med_cnt + 11) % 16] * 223 / 1969;
					last_detect = i;
				}

				med_cnt++;
				med_cnt = med_cnt % 16;

				data_cnt = 0;
			}
			//差分値が、振幅の取り得る値の最低値/3よりも大きい場合(decodeモード)
			else if (abs(dif) > amp_limit * th0 && search_step == 1) {
				med[med_cnt] = median(&in_buf[i - data_cnt + 1], data_cnt - 1) - base;
				//med[med_cnt] = in_buf[i - (int)(data_cnt / 2.0 ) ] - base;

				if (med_lv[(med_cnt + 15) % 16] == 0 &&
					med_lv[(med_cnt + 14) % 16] == 1) {
					out = out << quant_bit;
					if (med[med_cnt] > 0) {
						med[med_cnt] -= 2 * amp_limit;
					}

					if (med[med_cnt] < -amp_limit * th4) {
						out += 0b11;
					}
					else if (med[med_cnt] < -amp_limit * th3) {
						out += 0b10;
					}
					else if (med[med_cnt] < -amp_limit * th2) {
						out += 0b01;
					}
					else if (med[med_cnt] < -amp_limit * th1) {
						out += 0b00;

					}
					else {
						search_step = 0;
						med_cnt = 0;
						memset(med, 0, sizeof(int) * 16);
						memset(med_lv, 0, sizeof(int) * 16);

						continue;
					}

					out_cnt += quant_bit;
					bps_cnt += quant_bit;
				}
				if (med_lv[(med_cnt + 15) % 16] == 0 &&
					med_lv[(med_cnt + 14) % 16] == -1) {
					out = out << quant_bit;
					if (med[med_cnt] < 0) {
						med[med_cnt] += 2 * amp_limit;
					}

					if (med[med_cnt] > amp_limit * th4) {
						out += 0b11;
					}
					else if (med[med_cnt] > amp_limit * th3) {
						out += 0b10;
					}
					else if (med[med_cnt] > amp_limit * th2) {
						out += 0b01;
					}
					else if (med[med_cnt] > amp_limit * th1) {
						out += 0b00;
					}
					else {
						search_step = 0;
						med_cnt = 0;
						memset(med, 0, sizeof(int) * 16);
						memset(med_lv, 0, sizeof(int) * 16);

						continue;
					}

					out_cnt += quant_bit;
					bps_cnt += quant_bit;
				}

				if (med[med_cnt] > amp_limit * th1) {
					med_lv[med_cnt] = 1;
				}
				else if (med[med_cnt] < -amp_limit * th1) {
					med_lv[med_cnt] = -1;
				}
				else {
					med_lv[med_cnt] = 0;
					base = med[med_cnt] + base;
				}

				if (out_cnt % 16 == 0 && abs(med_lv[med_cnt]) == 1) {
					out_buf[(int)(out_cnt / 16) - 1] = out;

					out = 0;
				}

				med_cnt++;
				med_cnt = med_cnt % 16;

				data_cnt = 0;
				last_detect = i;
			}
		}
		else if (quant_bit == 1) {

			//入力の絶対値が、振幅の取り得る値の最低値/3よりも小さい場合(pilot_signal検出モード)
			if (abs(in_buf[i]) < amp_limit * th0 && base_flag == 1 && search_step == 0) {
				med[med_cnt] = maximum(&in_buf[i - data_cnt + 1], data_cnt == 0 ? 1 : data_cnt - 1, 0);
				//med[med_cnt] = in_buf[i - (int)(data_cnt / 2.0)];

				if (med[med_cnt] > amp_limit * th0) {
					med_lv[med_cnt] = 1;
				}
				else if (med[med_cnt] < -amp_limit * th0) {
					med_lv[med_cnt] = -1;
				}

				//check pilot signal
				if (med_lv[med_cnt] == -1 &&
					med_lv[(med_cnt + 15) % 16] == -1 &&
					med_lv[(med_cnt + 14) % 16] == 1 &&
					med_lv[(med_cnt + 13) % 16] == 1
					) {
					search_step = 1;
					amp_limit = med[(med_cnt + 14) % 16] * SAMPLE_MAX / 31504;
					//base = med[(med_cnt + 14) % 16];
					base = med[(med_cnt + 14) % 16] * 223 / 1969;
					last_detect = i;
				}

				med_cnt++;
				med_cnt = med_cnt % 16;

				data_cnt = 0;
				base_flag = 0;
			}
			else if (abs(in_buf[i]) >= amp_limit * th0 && search_step == 0) {
				base_flag = 1;
			}
			//入力の絶対値が、振幅の取り得る値の最低値/2よりも小さい場合(decodeモード)
			else if (abs(in_buf[i]-base) < amp_limit * th1 && base_flag == 1 && search_step == 1) {
				med[med_cnt] = maximum(&in_buf[i - data_cnt + 1], data_cnt - 1, base);
				//med[med_cnt] = in_buf[i - (int)(data_cnt / 2.0 ) ] - base;

				if (med_lv[(med_cnt + 15) % 16] == 1) {
					out = out << quant_bit;
					if (med[med_cnt] > 0) {
						out += 0b1;
						med[med_cnt] -= 2 * amp_limit;

						base += (int)((med[med_cnt] + amp_limit * (2910.0 / 2048.0)) * alpha);
					}
					else {
						out += 0b0;

						base += (int)((med[med_cnt] + amp_limit * (1746.0 / 2048.0)) * alpha);
					}

					out_cnt += quant_bit;
					bps_cnt += quant_bit;
				}
				if (med_lv[(med_cnt + 15) % 16] == -1) {
					out = out << quant_bit;
					if (med[med_cnt] < 0) {
						out += 0b1;
						med[med_cnt] += 2 * amp_limit;

						base += (int)((med[med_cnt] - amp_limit * (2910.0 / 2048.0)) * alpha);
					}
					else {
						out += 0b0;

						base += (int)((med[med_cnt] - amp_limit * (1746.0 / 2048.0)) * alpha);
					}

					out_cnt += quant_bit;
					bps_cnt += quant_bit;
				}

				if (med[med_cnt] > 0) {
					med_lv[med_cnt] = 1;
				}
				else if (med[med_cnt] < 0) {
					med_lv[med_cnt] = -1;
				}

				if (out_cnt % 16 == 0 && abs(med_lv[med_cnt]) == 1) {
					out_buf[(int)(out_cnt / 16) - 1] = out;

					out = 0;
				}

				med_cnt++;
				med_cnt = med_cnt % 16;

				data_cnt = 0;
				last_detect = i;
				base_flag = 0;
			}
			else if (abs(in_buf[i]-base) >= amp_limit * th1 && search_step == 1) {
				base_flag = 1;
			}
		}

		if (i % header->sample_rate == 0) {
			//printf("sec,bit=%8d,%5d\n", (int)(i / header->sample_rate), bps_cnt);
			printf("%d [%%] finished\r", (int)(i / (header->data_chunk.chunk_size / header->ch_num / 2.0) * 100.0 ));
			bps_cnt = 0;
		}

		if (out_cnt >= BIOS_SIZE_SHORT * 16) {
			break;
		}

	}
	if ((out_cnt % 16) != 0) {
		for (i = 0; i < 16 - (out_cnt % 16); i++) {
			out = out << 1;
			out += 0b0;
		}
		out_cnt += i;
		out_buf[(int)(out_cnt / 16) - 1] = out;
		out = 0;
	}
	fwrite(out_buf, sizeof(short), (int)(out_cnt / 16), fout);

	printf("100 [%%] finished\n");

	free(in_buf);
	free(out_buf);

	return ret;
}

int main(int argc, char** argv) {

	FILE* fin=NULL, * fout=NULL;
	int i;
	int io_flag = 0;
	READ_MODE mode = AM_MONO_HIGH;
	struct RIFF_HEADER header;

	if (argc < 3) {
		fprintf(stderr, "nbd_decoder.exe -m [mode] [in.wav] [out.bin]\n");
		fprintf(stderr, "mode:am_mono_high am_mono_low am_stereo_high am_stereo_low\n");
		fprintf(stderr, "   (default) %s\n", "am_mono_high");
		return -1;
	}
	for (i = 1; i < argc; i++) {
		if (argv[i][0] == '-') {
			switch (argv[i][1]) {
			case 'm':
				i++;
				if (i >= argc) {
					fprintf(stderr, "argument error\n");
					return -1;
				}
				if (!strcmp(argv[i], "am_mono_high")) {
					mode = AM_MONO_HIGH;
				}
				else if (!strcmp(argv[i], "am_mono_low")) {
					mode = AM_MONO_LOW;
				}
				else if (!strcmp(argv[i], "am_stereo_high")) {
					mode = AM_STEREO_HIGH;
				}
				else if (!strcmp(argv[i], "am_stereo_low")) {
					mode = AM_STEREO_LOW;
				}
				else {
					mode = DEF_MODE;
				}
				break;

			}
		}
		else {
			if (io_flag == 0) {
				if ((fin = fopen(argv[i], "rb")) == NULL) {
					fprintf(stderr, "input file open error:%s\n", argv[i]);
					if (fin != NULL) {
						fclose(fin);
						fin = NULL;
					}
					if (fout != NULL) {
						fclose(fout);
						fout = NULL;
					}
					return -1;
				}
				io_flag++;
			}
			else if (io_flag == 1) {
				if ((fout = fopen(argv[i], "wb")) == NULL) {
					fprintf(stderr, "output file open error:%s\n", argv[i]);
					if (fin != NULL) {
						fclose(fin);
						fin = NULL;
					}
					if (fout != NULL) {
						fclose(fout);
						fout = NULL;
					}
					return -1;
				}
			}
		}
	}

	read_wavheader(&header, fin);

	//Check Formats
	if (header.riff_identifier != RIFF_MARK ||
		header.format != FMT_WAVE ||
		header.fmt != AFMT_LPCM ||
		header.bit_depth != BDEPTH_16
		) {
		fprintf(stderr,"input file format error\n");
		if (fin != NULL) {
			fclose(fin);
			fin = NULL;
		}
		if (fout != NULL) {
			fclose(fout);
			fout = NULL;
		}
	}
	
	switch (mode) {
	case AM_MONO_HIGH:
		am_decode(&header, 2, fin, fout);
		break;

	case AM_MONO_LOW:
		am_decode(&header, 1, fin, fout);
		break;
		
	case AM_STEREO_HIGH:
		fprintf(stderr, "am_stereo mode is unimplemented\n");
		break;

	case AM_STEREO_LOW:
		fprintf(stderr, "am_stereo mode is unimplemented\n");
		break;

	}
	

	if (fin != NULL) {
		fclose(fin);
		fin = NULL;
	}
	if (fout != NULL) {
		fclose(fout);
		fout = NULL;
	}

	return 0;
}

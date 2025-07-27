#include <bits/stdc++.h>
using namespace std;

void calculate_key(uint8_t *input, uint8_t *output) {
	output[0] = input[0] ^ input[1];
	output[1] = input[1] + input[2];
	output[2] = input[2] ^ input[3];
	output[3] = input[3] + input[0];
	output[4] = input[4] & 0xF0;
	output[5] = input[5] & 0x0F;
}

int main(){
	int N;
	uint8_t seed[64];
	uint8_t key[64];
	cin >> N;
    for(int i = 0; i < N; i++){
		scanf("%2hhX",&seed[i]);
	}
    calculate_key(seed,key);
	for(int i = 0; i < N; i++){
		printf("%02X ",key[i]);
	}
	return 0;
}
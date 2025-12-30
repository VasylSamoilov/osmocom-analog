
/* Get the negotiated sample rate */
int sdr_get_samplerate(void *inst)
{
	sdr_t *sdr = (sdr_t *)inst;
	if (!sdr)
		return 0;
	return sdr->samplerate;
}

/* Calculate optimal channel rate based on master rate and bandwidth */
int sdr_calculate_optimal_rate(int master_rate, double bandwidth)
{
	int rate = master_rate;
	
	/* We need at least bandwidth + margin.
	 * Decimate by powers of 2 until we hit the limit. */
	while (rate > bandwidth * 1.2 && (rate % 2) == 0) {
		int next_rate = rate / 2;
		if (next_rate < bandwidth * 1.1)
			break; /* Too low */
		rate = next_rate;
	}
	
	return rate;
}

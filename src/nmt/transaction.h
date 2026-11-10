
/* info about subscriber */
typedef struct nmt_subscriber {
	/* NOTE: country must be followed by number, so both represent a string */
	char			country;		/* country digit */
	char			number[7];		/* phone suffix */
	char			password[4];		/* phone's password + '\0' */
	int			coinbox;		/* phone is a coinbox and accept tariff information */
} nmt_subscriber_t;

/* Where a call to the MS is while it is on the calling channels (the calling
 * channel carousel in nmt.c): paged (2a), answered and getting the TC (2b), or
 * answered and waiting for a free TC (2f/2c, then standby). */
enum cc_phase {
	CC_PHASE_NONE = 0,	/* not on the calling channels */
	CC_PHASE_DEFERRED,	/* battery saving: waiting for the group's wake window */
	CC_PHASE_PAGE,		/* paged, waiting for 10a/10d/11b */
	CC_PHASE_ORDER,		/* answered, channel order 2b to send */
	CC_PHASE_QUEUE,		/* answered, no free TC: queued */
};

#define NMT_MAX_CC	16	/* calling channels a page can go on */

/* transaction node */
typedef struct transaction {
	struct transaction	*next;			/* pointer to next node in list */
	nmt_t			*nmt;			/* pointer to nmt instance, if bound to a channel */
	int			callref;		/* callref for transaction */
	struct nmt_subscriber	subscriber;
	struct osmo_timer_list		timer;		/* page tries, battery saving wait, 2T order deadline, queue */
	int			page_try;		/* number of paging try */
	int			page_deferred;		/* battery saving: page is waiting for the group's wake window */
	int			page_waited;		/* battery saving: the wake window was waited for */
	enum cc_phase		cc_phase;		/* where the call is on the calling channels */
	nmt_t			*page_cc[NMT_MAX_CC];	/* calling channels that page the MS */
	int			page_cc_num;
	uint8_t			page_due[NMT_MAX_CC];	/* ... whose 2a of this try is still to go out */
	nmt_t			*cc;			/* calling channel of the answer (order, queue) */
	int			queue_info_left;	/* 2f/2c frames still to send */
	int			repaged;		/* paged again: a TC became free for it */
	int			coinbox_answer_pending;	/* coin-box: far end answered, frame 5b (tariff) must be sent next */
	int			priority_mo;		/* PMS: MS-initiated MO call via priority seizure (4.4.1.7); no network callref */
	int			pending_power;		/* power-change (4.4.1.8): -1 = none, else new ms_power to request via 3a */
	char			recall_code[64];	/* register recall (4.4.1.10): digits collected via 14a/14b */
	int			recall_code_len;	/* number of recall digits collected so far */
	int			recall_mft_num;		/* odd/even parity counter for 14a/14b sequencing during recall */
	int			priority;		/* MS replied with priority seizure (11b): queue with 2c, not 2f */

	/* caller ID */
	char			caller_id[33];		/* caller id digits */
	enum number_type	caller_type;		/* caller id type */

	/* DMS/SMS */
	int			dms_call;		/* indicates to use DMS (used for SMS) */
	char			sms_string[256];	/* current message to deliver (may contain any byte, see sms_len) */
	int			sms_len;		/* length of sms_string in bytes (so an embedded 0x00 is kept) */

	/* MWI (Message Waiting Indicator)
	 * See docs/NMT_MWI.md for details on MWI implementation.
	 *
	 * mwi_flags bits:
	 *   bit 0 (0x01) = SMS message waiting
	 *   bit 1 (0x02) = Voice mail waiting
	 *   bit 2 (0x04) = Fax waiting
	 *   bit 3 (0x08) = E-mail waiting
	 *   bit 4 (0x10) = Data waiting
	 */
	int			mwi_call;		/* 1 = this is an MWI-only delivery call */
	uint8_t			mwi_flags;		/* indicator flags for current frame 5c */
	uint8_t			mwi_pending;		/* accumulated flags set during call */
} transaction_t;

transaction_t *create_transaction(struct nmt_subscriber *subscriber);
void destroy_transaction(transaction_t *trans);
transaction_t *get_transaction_by_callref(int callref);
transaction_t *get_transaction_by_number(struct nmt_subscriber *subscr);
transaction_t *get_transaction_list(void);



/* info about subscriber */
typedef struct nmt_subscriber {
	/* NOTE: country must be followed by number, so both represent a string */
	char			country;		/* country digit */
	char			number[7];		/* phone suffix */
	char			password[4];		/* phone's password + '\0' */
	int			coinbox;		/* phone is a coinbox and accept tariff information */
} nmt_subscriber_t;

/* transaction node */
typedef struct transaction {
	struct transaction	*next;			/* pointer to next node in list */
	nmt_t			*nmt;			/* pointer to nmt instance, if bound to a channel */
	int			callref;		/* callref for transaction */
	struct nmt_subscriber	subscriber;
	struct osmo_timer_list		timer;
	int			page_try;		/* number of paging try */

	/* caller ID */
	char			caller_id[33];		/* caller id digits */
	enum number_type	caller_type;		/* caller id type */

	/* DMS/SMS */
	int			dms_call;		/* indicates to use DMS (used for SMS) */
	char			sms_string[256];	/* current string to deliver */

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


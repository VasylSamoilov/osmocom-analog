
enum amps_trans_state {
	TRANS_NULL = 0,
	TRANS_REGISTER_ACK,		/* attach request received, waiting to ack */
	TRANS_REGISTER_ACK_SEND,	/* attach request received, sending ack */
	TRANS_CALL_MO_ASSIGN,		/* assigning channel, waiting to send */
	TRANS_CALL_MO_ASSIGN_SEND,	/* assigning channel, sending assignment */
	TRANS_CALL_MO_ASSIGN_CONFIRM,	/* assignment sent, waiting for confirm (SAT) */
	TRANS_CALL_MT_ASSIGN,		/* assigning channel, waiting to send */
	TRANS_CALL_MT_ASSIGN_SEND,	/* assigning channel, sending assignment */
	TRANS_CALL_MT_ASSIGN_CONFIRM,	/* assignment sent, waiting for confirm (SAT) */
	TRANS_CALL_MT_ALERT,		/* ringing the phone, waiting to send alert */
	TRANS_CALL_MT_ALERT_SEND,	/* ringing the phone, sending alert */
	TRANS_CALL_MT_ALERT_CONFIRM,	/* ringing the phone, signaling tone is received */
	TRANS_CALL_MT_ANSWER_WAIT,	/* ringing the phone, waiting for the phone to answer */
	TRANS_CALL_REJECT,		/* rejecting channel, waiting to send */
	TRANS_CALL_REJECT_SEND,		/* rejecting channel, sending reject */
	TRANS_CALL,			/* active call */
	TRANS_CALL_CHANGE_POWER,	/* changing power level, waiting to send */
	TRANS_CALL_CHANGE_POWER_SEND,	/* changing power level, sending */
	TRANS_CALL_FLASH_INFO,		/* flash with info, waiting to send */
	TRANS_CALL_FLASH_INFO_SEND,	/* flash with info, sending */
	TRANS_CALL_PCI_QUERY,		/* PCI query on FVC, waiting to send */
	TRANS_CALL_PCI_QUERY_SEND,	/* PCI query on FVC, sending */
	TRANS_CALL_AUDIT,		/* Audit order on FVC, waiting to send */
	TRANS_CALL_AUDIT_SEND,		/* Audit order, sending */
	TRANS_CALL_RELEASE,		/* release call towards phone, waiting to send */
	TRANS_CALL_RELEASE_SEND,	/* release call towards phone, sending release */
	TRANS_CALL_REORDER,		/* sending Reorder order (Order 4), waiting to send */
	TRANS_CALL_REORDER_SEND,	/* sending Reorder order (Order 4), sending */
	TRANS_CALL_MWI,			/* sending Message Waiting order (Order 5), waiting to send */
	TRANS_CALL_MWI_SEND,		/* sending Message Waiting order (Order 5), sending */
	TRANS_CALL_STOP_ALERT,		/* sending Stop Alert order (Order 6), waiting to send */
	TRANS_CALL_STOP_ALERT_SEND,	/* sending Stop Alert order (Order 6), sending */
	TRANS_CALL_INTERCEPT,		/* sending Intercept order (Order 9), waiting to send */
	TRANS_CALL_INTERCEPT_SEND,	/* sending Intercept order (Order 9), sending */
	TRANS_CALL_MAINTENANCE,		/* sending Maintenance order (Order 10), waiting to send */
	TRANS_CALL_MAINTENANCE_SEND,	/* sending Maintenance order (Order 10), waiting for ST */
	TRANS_CALL_ABB_ALERT,		/* sending Abbreviated Alert (Order 1, ORDQ=1), waiting to send */
	TRANS_CALL_ABB_ALERT_SEND,	/* sending Abbreviated Alert (Order 1, ORDQ=1), waiting for ST */
	TRANS_CALL_ESN_REQUEST,		/* sending ESN request (Order 15), waiting to send */
	TRANS_CALL_ESN_REQUEST_SEND,	/* sending ESN request (Order 15), waiting for response */
	TRANS_CALL_ESN_REPLY,		/* ESN response received (Order 15) */
	TRANS_CALL_DIGITS_REQUEST,	/* sending Called-Address request (Order 8), waiting to send */
	TRANS_CALL_DIGITS_REQUEST_SEND,	/* sending Called-Address request (Order 8), waiting for response */
	TRANS_CALL_DIGITS_REPLY,	/* Called-Address response received (Order 8) */
	TRANS_CALL_LOCAL_CONTROL,	/* sending Local Control order (Order 30), waiting to send */
	TRANS_CALL_LOCAL_CONTROL_SEND,	/* sending Local Control order (Order 30), sending */
	TRANS_CALL_HANDOFF,		/* sending Handoff message on FVC, waiting to send */
	TRANS_CALL_HANDOFF_SEND,	/* sending Handoff message on FVC, waiting for ST */
	TRANS_CALL_HANDOFF_CONFIRM,	/* Handoff ST received, moving to new channel */
	TRANS_DIRECTED_RETRY,		/* sending Directed Retry order (Order 12) on FOCC, waiting to send */
	TRANS_DIRECTED_RETRY_SEND,	/* sending Directed Retry order (Order 12) on FOCC, sending */
	TRANS_AUDIT,			/* Audit order on FOCC, waiting to send */
	TRANS_AUDIT_SEND,		/* Audit order on FOCC, sending */
	TRANS_PCI,			/* PCI query on FOCC, waiting to send */
	TRANS_PCI_SEND,			/* PCI query on FOCC, sending */
	TRANS_MWI,			/* MWI order on FOCC, waiting to send */
	TRANS_MWI_SEND,			/* MWI order on FOCC, sending */
	TRANS_PAGE,			/* paging phone, waiting to send */
	TRANS_PAGE_SEND,		/* paging phone, sending page order */
	TRANS_PAGE_REPLY,		/* waitring for paging reply */
	TRANS_SILENT_PAGE,		/* silent page: paging phone for maintenance test, waiting to send */
	TRANS_SILENT_PAGE_SEND,		/* silent page: sending page order */
	TRANS_SILENT_PAGE_REPLY,	/* silent page: waiting for paging reply */
	TRANS_SILENT_PAGE_ASSIGN,	/* silent page: assigning voice channel, waiting to send */
	TRANS_SILENT_PAGE_ASSIGN_SEND,	/* silent page: sending channel assignment */
	TRANS_SILENT_PAGE_ASSIGN_CONFIRM, /* silent page: waiting for SAT confirmation */
	TRANS_SILENT_PAGE_MAINTENANCE,	/* silent page: sending Maintenance order, waiting to send */
	TRANS_SILENT_PAGE_MAINTENANCE_SEND, /* silent page: Maintenance sent, waiting for ST */
};

typedef struct transaction {
	struct transaction	*next;			/* pointer to next node in list */
	amps_t			*amps;			/* pointer to amps instance */
	int			callref;		/* call reference */
	int			page_retry;		/* current number of paging (re)try */
	uint32_t		min1;			/* current station ID (2 values) */
	uint16_t		min2;
	uint32_t		esn;			/* ESN */
	uint8_t			msg_type;		/* message type (3 values) */
	uint8_t			ordq;
	uint8_t			order;
	uint16_t		chan;			/* channel to assign */
	int			alert_retry;		/* current number of alter order (re)try */
	char			caller_id[33];		/* id of calling phone */
	char			dialing[33];		/* number dialed by the phone */
	enum amps_trans_state	state;			/* state of transaction */
	enum amps_trans_state	mwi_return_state;	/* state to return to after MWI order */
	struct osmo_timer_list		timer;			/* for varous timeouts */
	struct osmo_timer_list		flash_timer;		/* timer for auto Order 8 after flash */
	int			sat_detected;		/* state if we detected SAT */
	int			dtx;			/* if set, DTX is used with this call */
	double			st_start_time;		/* time when ST was first detected */
	double			flash_time;		/* time when last flash was detected (for auto Order 8) */
	uint8_t			current_vmac;		/* current VMAC value (0-7) */
	uint8_t			max_vmac;		/* maximum VMAC allowed (from ms_power/limit) */
	double			sat_level_avg;		/* running average of SAT level */
	int			vmac_adjust_count;	/* samples since last VMAC adjustment */
	int			vmac_grace_count;	/* grace period samples after power change */
	int			signal_pitch;		/* pitch of alerting signal (0-3) */
	int			signal_cadence;		/* cadence of alerting signal (0-63) */
	int			presentation_indicator;	/* presentation indicator (0-3 or -1 for default) */
	int			screening_indicator;	/* screening indicator (0-3 or -1 for default) */
	/* Directed Retry fields */
	int			retry_channels[6];	/* channel positions for Directed Retry (up to 6) */
	int			retry_num_channels;	/* number of channels in retry_channels */
	int			retry_last_try;		/* 1 = last try (ORDQ=1), 0 = not last try (ORDQ=0) */
	/* Handoff fields */
	int			handoff_channel;	/* target channel for handoff */
	int			handoff_scc;		/* target SAT color code for handoff */
} transaction_t;

transaction_t *create_transaction(amps_t *amps, enum amps_trans_state trans_state, uint32_t min1, uint16_t min2, uint32_t esn, uint8_t msg_type, uint8_t ordq, uint8_t order, uint16_t chan);
void destroy_transaction(transaction_t *trans);
void link_transaction(transaction_t *trans, amps_t *amps);
void unlink_transaction(transaction_t *trans);
transaction_t *search_transaction(amps_t *amps, uint32_t state_mask);
transaction_t *search_transaction_number(amps_t *amps, uint32_t min1, uint16_t min2);
transaction_t *search_transaction_callref(amps_t *amps, int callref);
void trans_new_state(transaction_t *trans, int state);
void amps_flush_other_transactions(amps_t *amps, transaction_t *trans);
void transaction_timeout(void *data);
const char *trans_short_state_name(int state);


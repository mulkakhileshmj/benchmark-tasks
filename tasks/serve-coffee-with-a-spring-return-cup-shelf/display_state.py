"""Display-only schema. No score or alternate liquid ledger lives here."""
GOOD='cm_mug_good'
CRACKED='cm_mug_cracked'


def stream_on(state):return state['brew_step'] is not None


def new_ledger():
    return dict(step=0,brew_step=None,target=None,level={GOOD:0.,CRACKED:0.},
                steam_mug=None,steam_until=-1,tiles=[])

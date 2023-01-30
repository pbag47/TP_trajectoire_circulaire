from agent_class import Agent


def init_agents():
    cf_radio_id = {'01': 'radio://0/81/2M/E7E7E7E701',
                   '02': 'radio://1/82/2M/E7E7E7E702',
                   '03': 'radio://2/83/2M/E7E7E7E703',
                   '04': 'radio://3/84/2M/E7E7E7E704',
                   '05': 'radio://0/85/2M/E7E7E7E705',
                   '06': 'radio://1/86/2M/E7E7E7E706',
                   '07': 'radio://2/87/2M/E7E7E7E707',
                   '08': 'radio://3/88/2M/E7E7E7E708',
                   '09': 'radio://0/89/2M/E7E7E7E709',
                   '10': 'radio://1/90/2M/E7E7E7E710'}

    cf1 = Agent('cf1', cf_radio_id['01'])
    cf2 = Agent('cf2', cf_radio_id['02'])
    cf3 = Agent('cf3', cf_radio_id['03'])
    cf4 = Agent('cf4', cf_radio_id['04'])
    cf5 = Agent('cf5', cf_radio_id['05'])
    cf6 = Agent('cf6', cf_radio_id['06'])
    cf7 = Agent('cf7', cf_radio_id['07'])
    cf8 = Agent('cf8', cf_radio_id['08'])
    cf9 = Agent('cf9', cf_radio_id['09'])
    cf10 = Agent('cf10', cf_radio_id['10'])

    uavs = [
        cf1,
        cf2,
        cf3,
        cf4,
        cf5,
        cf6,
        cf7,
        cf8,
        cf9,
        cf10,
    ]
    return uavs

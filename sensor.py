# my_components/qmc5883p/sensor.py
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import CONF_ID, CONF_ADDRESS, CONF_UPDATE_INTERVAL

DEPENDENCIES = ["i2c"]

qmc_ns = cg.esphome_ns.namespace("qmc5883p")
QMC5883PSensor = qmc_ns.class_("QMC5883PSensor", cg.PollingComponent, i2c.I2CDevice)

CONF_FIELD_STRENGTH_X = "field_strength_x"
CONF_FIELD_STRENGTH_Y = "field_strength_y"
CONF_FIELD_STRENGTH_Z = "field_strength_z"
CONF_FIELD_STRENGTH   = "field_strength"
CONF_OFFSETS          = "offsets"    # [ox,oy,oz]
CONF_MATRIX           = "matrix"     # 9 floats row-major
CONF_ROTATION         = "rotation"   # 0,90,180,270
CONF_AXES_MAP         = "axes_map"   # ["x","y","z"] / with signs

AXIS_LUT = {"x":1, "y":2, "z":3, "-x":-1, "-y":-2, "-z":-3}

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(QMC5883PSensor),

    cv.Optional(CONF_ADDRESS, default=0x2C): cv.i2c_address,
    cv.Optional(CONF_UPDATE_INTERVAL, default="1000ms"): cv.update_interval,

    cv.Required(CONF_FIELD_STRENGTH_X): sensor.sensor_schema(accuracy_decimals=0),
    cv.Required(CONF_FIELD_STRENGTH_Y): sensor.sensor_schema(accuracy_decimals=0),
    cv.Required(CONF_FIELD_STRENGTH_Z): sensor.sensor_schema(accuracy_decimals=0),
    cv.Optional(CONF_FIELD_STRENGTH):   sensor.sensor_schema(accuracy_decimals=1),

    cv.Optional(CONF_OFFSETS, default=[0.0, 0.0, 0.0]):
        cv.All(cv.ensure_list(cv.float_), cv.Length(min=3, max=3)),
    cv.Optional(CONF_MATRIX, default=[1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]):
        cv.All(cv.ensure_list(cv.float_), cv.Length(min=9, max=9)),
    cv.Optional(CONF_ROTATION, default=0): cv.one_of(0, 90, 180, 270, int=True),

    cv.Optional(CONF_AXES_MAP, default=["x", "y", "z"]):
        cv.All(
            cv.ensure_list(cv.one_of("x","y","z","-x","-y","-z", lower=True)),
            cv.Length(min=3, max=3)
        ),
}).extend(i2c.i2c_device_schema(0x2C))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    xs = await sensor.new_sensor(config[CONF_FIELD_STRENGTH_X])
    ys = await sensor.new_sensor(config[CONF_FIELD_STRENGTH_Y])
    zs = await sensor.new_sensor(config[CONF_FIELD_STRENGTH_Z])
    cg.add(var.set_output_sensors(xs, ys, zs))

    if CONF_FIELD_STRENGTH in config:
        fs = await sensor.new_sensor(config[CONF_FIELD_STRENGTH])
        cg.add(var.set_field_sensor(fs))

    offs = config[CONF_OFFSETS]
    cg.add(var.set_offsets(offs[0], offs[1], offs[2]))

    mat = config[CONF_MATRIX]
    cg.add(var.set_matrix(
        mat[0], mat[1], mat[2],
        mat[3], mat[4], mat[5],
        mat[6], mat[7], mat[8]
    ))

    cg.add(var.set_rotation(config[CONF_ROTATION]))

    amap = [AXIS_LUT[s] for s in config[CONF_AXES_MAP]]
    cg.add(var.set_axes_map(amap[0], amap[1], amap[2]))

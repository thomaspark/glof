import * as Phaser from 'phaser';

const config = {
    type: Phaser.CANVAS,
    width: 800,
    height: 600,
    canvas: document.getElementById('app'),
    resolution: window.devicePixelRatio,
    backgroundColor: '#16565d',
    physics: {
        default: 'matter',
        matter: {
            gravity: { y: 1 },
            debug: false
        }
    },
    scene: {
        preload: preload,
        create: create,
        update: update
    }
};

const game = new Phaser.Game(config);

let club, clubHead, lowerarm, upperarm, shoulder, ball, tee, ramen;
let cursors, zKey, xKey, aKey, rKey, sKey, qKey, wKey, spaceKey, escKey; // Declare variables for the keys
let score, best, maxText, retryText;
let max = 0, gameOver = false, hit = false;
let groundGraphics, ballGraphics, clubGraphics, upperarmGraphics,
    lowerarmGraphics, background, teeGraphics, maxGraphics, ramenGraphics;
let startX, startTeeHeight;
const bucketX = yardsToPixels(135);

function preload() {
    this.load.image('controls', './controls.png');
    this.load.image('upperarm', './upperarm.png');
    this.load.image('lowerarm', './lowerarm.png');
    this.load.image('club', './club.png');
    this.load.image('ball', './ball.png');
    this.load.image('tee', './tee.png');
    this.load.image('body', './body.png');
    this.load.image('head', './head.png');
    this.load.image('flag', './flag.png');
    this.load.image('ramen', './ramen.png');
    this.load.image('bucket', './bucket.png');
    this.load.image('fox', './fox.png');
    this.load.image('bear', './bear.png');
    this.load.image('rhino', './rhino.png');
    this.load.image('bag', './bag.png');
    this.load.image('max', './max.png');
    this.load.image('trophy', './trophy.png');
    this.load.audio('shot', './shot.wav');
    this.load.audio('thud', './thud.mp3');
    this.load.audio('wood', './wood.mp3');
    this.load.image('background', './background.png');
}

function create() {
    gameOver = false;
    hit = false;
    
    this.cameras.main.setBackgroundColor("#62A6C2");
    background = this.add.tileSprite(50, 100, 800, 140, 'background');
    background.tileScaleY = 0.5;

    const groundHeight = 400;
    const groundWidth = config.width * 50;
    const groundX = config.width / 2;
    const groundY = config.height - (80 - groundHeight)/2;
    const ground = this.matter.add.rectangle(groundX, groundY, groundWidth, groundHeight, {
        isStatic: true,
        restitution: 0.2, // Bounciness
        friction: 0.5,
        label: 'ground'
    });

    groundGraphics = this.add.graphics();
    groundGraphics.fillStyle(0x15565D, 1);
    groundGraphics.fillRect(-1 * groundWidth, groundY - groundHeight/2, groundWidth * 2, groundHeight); // Match ground dimensions

    let bagGraphics = this.add.sprite(yardsToPixels(-35), 375, 'bag');
    bagGraphics.setScale(0.4);

    let trophyGraphics = this.add.sprite(yardsToPixels(-110), 307, 'trophy');
    trophyGraphics.setScale(0.4);

    let bucketLeft = this.matter.add.rectangle(bucketX - 52, 530, 5, 73, {
        isStatic: true,
        restitution: 0.5,
        friction: 0.1,
        density: 1,
        angle: -Math.PI / 7,
        label: 'bucketLeft'
    });

    let bucketRight = this.matter.add.rectangle(bucketX + 45, 518, 5, 113, {
        isStatic: true,
        restitution: 0.5,
        friction: 0.1,
        density: 1,
        angle: -Math.PI / 12.5,
        label: 'bucketRight'
    });

    let bucketBottom = this.matter.add.rectangle(bucketX + 6, 561, 100, 5, {
        isStatic: true,
        restitution: 0.5,
        friction: 0.1,
        density: 1,
        label: 'bucketBottom'
    });


    ramen = this.matter.add.rectangle(yardsToPixels(10), 519, 80, 80, {
        restitution: 1, // Bounciness
        friction: 0.1,
        density: 0.00001,
        label: 'ramen'
    });

    ramenGraphics = this.add.sprite(yardsToPixels(10), 519, 'ramen');
    ramenGraphics.setScale(0.115);

    let foxGraphics = this.add.sprite(yardsToPixels(310), 488, 'fox');
    foxGraphics.setScale(0.25);

    let bearGraphics = this.add.sprite(yardsToPixels(460), 448, 'bear');
    bearGraphics.setScale(0.25);

    let rhinoGraphics = this.add.sprite(yardsToPixels(-310), 414, 'rhino');
    rhinoGraphics.setScale(0.3);

    let bodyGraphics = this.add.sprite(340, 340, 'body');
    bodyGraphics.setScale(0.33);
    
    let controlsGraphics = this.add.sprite(120, 270, 'controls');
    controlsGraphics.setScale(0.28);

    club = this.matter.add.rectangle(340, 502, 10, 200, {
        restitution: 0.5, // Bounciness
        friction: 0.1,
        density: 1,
        label: 'club'
    });

    clubGraphics = this.add.sprite(20, 20, 'club');
    clubGraphics.setScale(0.33);

    clubHead = this.matter.add.polygon(350, 515, 3, 10, {
        restitution: 0.5, // Bounciness
        friction: 0.1,
        density: 1,
        label: 'clubHead'
    });

    let clubComposite = this.matter.composite.create();
    this.matter.composite.add(clubComposite, [club, clubHead]);

    lowerarm = this.matter.add.rectangle(340, 310, 10, 100, {
        restitution: 0, // Bounciness
        friction: 0,
        density: 9,
        angularDamping: 0.5,
        label: 'lowerarm'
    });

    lowerarmGraphics = this.add.sprite(20, 20, 'lowerarm');
    lowerarmGraphics.setScale(0.29);

    upperarm = this.matter.add.rectangle(340, 200, 10, 100, {
        // isKinematic: true,
        restitution: 0, // Bounciness
        density: 90,
        angularDamping: 0.5,
        friction: 0.1,
        label: 'upperarm'
    });

    upperarmGraphics = this.add.sprite(20, 20, 'upperarm');
    upperarmGraphics.setScale(0.33);


    shoulder = this.matter.add.rectangle(340, 140, 10, 10, {
        isStatic: true,
        restitution: 0.5, // Bounciness
        density: 900,
        angularDamping: 0.5,
        friction: 0.1,
        label: 'lowerarm'
    });

    let headGraphics = this.add.sprite(336, 101, 'head');
    headGraphics.setScale(0.33);

    var wristConstraint = this.matter.add.constraint(club, lowerarm, 0, 0.9, {
        pointA: {
            x: 0,
            y: -100,
        },
        pointB: {
            x: 0,
            y: 50,
        },
    });

    var elbowConstraintLeft = this.matter.add.constraint(lowerarm, upperarm, 10, 0.1, {
        pointA: {
            x: -30,
            y: -50,
        },
        pointB: {
            x: -30,
            y: 40,
        },
    });

    var elbowConstraintRight = this.matter.add.constraint(lowerarm, upperarm, 10, 0.1, {
        pointA: {
            x: 30,
            y: -50,
        },
        pointB: {
            x: 30,
            y: 40,
        },
    });

    var shoulderConstraint = this.matter.add.constraint(upperarm, shoulder, 2, 0.9, {
        pointA: {
            x: 0,
            y: -50,
        },
        pointB: {
            x: 0,
            y: 0,
        },
    });

    var clubConstraint = this.matter.add.constraint(club, clubHead, 0, 1, {
        pointA: {
            x: 5,
            y: 95,
        },
        pointB: {
            x: 0,
            y: 0,
        },
    });

    tee = this.matter.add.rectangle(startX ? startX : 400, 552, 15, startTeeHeight ? startTeeHeight : 18, {
        restitution: 0,
        friction: 1,
        frictionAir: 0.1,
        density: 0.1,
        label: 'tee'
    });
    
    teeGraphics = this.add.sprite(15, startTeeHeight ? startTeeHeight : 18, 'tee');
    teeGraphics.setScale(0.42, 0.5);

    ball = this.matter.add.circle(startX ? startX : 400, startTeeHeight ? 541 - startTeeHeight : 523, 20, {
        restitution: 1,
        friction: 1,
        frictionAir: 0.01,
        label: 'ball'
    });

    ballGraphics = this.add.sprite(20, 20, 'ball');
    ballGraphics.setScale(0.6);

    let flagY = 451;
    for (let i = -1000; i <= 1000; i += 50) {
        if (i === 0) continue;

        let flagX = yardsToPixels(i);

        let flagGraphics = this.add.sprite(flagX, flagY, 'flag');
        flagGraphics.setScale(0.28);

        this.add.text(flagX + 3, flagY - 76, i, {
            fontFamily: "ui-monospace, 'Cascadia Code', 'Source Code Pro', Menlo, Consolas, 'DejaVu Sans Mono', monospace",
            fontSize: '14px',
            fontStyle: '800',
            fill: '#fff'
        });
    }

    if (max > 0) {
        let flagX = yardsToPixels(max);
        maxGraphics = this.add.sprite(yardsToPixels(max), flagY, 'max');
        maxText = this.add.text(flagX + 3, flagY - 76, max, {
            fontFamily: "ui-monospace, 'Cascadia Code', 'Source Code Pro', Menlo, Consolas, 'DejaVu Sans Mono', monospace",
            fontSize: '14px',
            fontStyle: '800',
            fill: '#fff'
        });
    } else {
        maxGraphics = this.add.sprite(yardsToPixels(-999), flagY, 'max');
        maxText = this.add.text(-999 + 3, flagY - 76, '', {
            fontFamily: "ui-monospace, 'Cascadia Code', 'Source Code Pro', Menlo, Consolas, 'DejaVu Sans Mono', monospace",
            fontSize: '14px',
            fontStyle: '800',
            fill: '#fff'
        });
    }
    maxGraphics.setScale(0.28);


    let bucketGraphics = this.add.sprite(bucketX, 528, 'bucket');
    bucketGraphics.setScale(0.3);
    bucketGraphics.rotation = -Math.PI / 9;

    // Enable keyboard input
    cursors = this.input.keyboard.createCursorKeys();

    zKey = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.Z);
    xKey = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.X);
    aKey = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.A);
    sKey = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.S);
    qKey = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.Q);
    wKey = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.W);
    rKey = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.R);
    escKey = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.ESC);
    spaceKey = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);

    var shot = this.sound.add('shot');
    var thud = this.sound.add('thud');
    var wood = this.sound.add('wood');
    this.matter.world.on('collisionstart', function (event, bodyA, bodyB) {
        if (
            (bodyA.label === 'club' && bodyB.label === 'ball') || 
            (bodyA.label === 'clubHead' && bodyB.label === 'ball') ||
            (bodyA.label === 'club' && bodyB.label === 'tee' && Math.abs(pixelsToYards(ball.position.x)) < 5) || 
            (bodyA.label === 'clubHead' && bodyB.label === 'tee' && Math.abs(pixelsToYards(ball.position.x)) < 5)
        ) {
            hit = true;
            shot.play({
                seek: 0.4
            });
        } else if ( 
            (bodyA.label === 'ground' && bodyB.label === 'ball') 
        ) {
            thud.play();
        } else if ( 
            (bodyA.label === 'bucketLeft' && bodyB.label === 'ball') ||
            (bodyA.label === 'bucketRight' && bodyB.label === 'ball') ||
            (bodyA.label === 'bucketBottom' && bodyB.label === 'ball')
        ) {
            wood.play();
        }
    });


    score = this.add.text(16, 16, '0 yards', {
        fontFamily: "ui-monospace, 'Cascadia Code', 'Source Code Pro', Menlo, Consolas, 'DejaVu Sans Mono', monospace",
        fontSize: '24px',
        fontStyle: '900',
        fill: '#fff'
    });

    score.y = 12;

    let bestText = max > 0 ? `Best: ${max}` : '';
    best = this.add.text(700, 20, bestText, {
        fontFamily: "ui-monospace, 'Cascadia Code', 'Source Code Pro', Menlo, Consolas, 'DejaVu Sans Mono', monospace",
        fontSize: '14px',
        fontStyle: '800',
        fill: '#fff'
    });

    retryText = this.add.text(250, 250, 'You\'re ace!\nPress Spacebar to continue.', {
        fontFamily: "ui-monospace, 'Cascadia Code', 'Source Code Pro', Menlo, Consolas, 'DejaVu Sans Mono', monospace",
        fontSize: '18px',
        fontStyle: '800',
        align: 'center',
        fill: '#fff'
    });
    retryText.visible = false;
}

function update() {
    if (!hit && cursors.left.isDown) {
        startX = startX ? Math.max(startX - 1, 380) : 399;
        this.scene.restart();
    }

    if (!hit && cursors.right.isDown) {
        startX = startX ? Math.min(startX + 1, 420) : 399;
        this.scene.restart();
    }

    if (!hit && cursors.down.isDown) {
        startTeeHeight = startTeeHeight ? Math.max(startTeeHeight - 1, 1) : 18;
        this.scene.restart();
    }

    if (!hit && cursors.up.isDown) {
        startTeeHeight = startTeeHeight ? Math.min(startTeeHeight + 1, 30) : 18;
        this.scene.restart();
    }

    if (zKey.isDown) {
        if ((club.angle - lowerarm.angle) % Math.PI > Math.PI / 2) {
            this.matter.body.setAngularVelocity(club, 0);
        } else {
            this.matter.body.setAngularVelocity(club, 0.15);
        }
    }
    
    if (xKey.isDown) {
        if ((club.angle - lowerarm.angle) % Math.PI < Math.PI / -2) {
            this.matter.body.setAngularVelocity(club, 0);
        }   else { 
            this.matter.body.setAngularVelocity(club, -0.15);
        }
    }
    
    if (aKey.isDown) {
        this.matter.body.setAngularVelocity(lowerarm, 0.05); // Rotate clockwise
    }
    
    if (sKey.isDown) {
        this.matter.body.setAngularVelocity(lowerarm, -0.10); // Rotate counter-clockwise
    }
    
    if (qKey.isDown) {
        this.matter.body.setAngularVelocity(upperarm, 0.05); // Rotate clockwise
    }
    
    if (wKey.isDown) {
        this.matter.body.setAngularVelocity(upperarm, -0.1); // Rotate counter-clockwise
    }
    
    if (rKey.isDown) {
        this.scene.restart();
    }
    
    if (spaceKey.isDown && gameOver) {
        this.scene.restart();
    }
    
    if (escKey.isDown) {
        startX = undefined;
        startTeeHeight = undefined;
        this.scene.restart();
    }
    
    if (!zKey.isDown && !xKey.isDown) {
        // this.matter.body.setAngularVelocity(club, 0);
    }

    this.cameras.main.scrollX = ball.position.x - 800 / 2;

    score.setText(`${pixelsToYards(ball.position.x)} yards`);
    score.x = ball.position.x - 60;

    best.x = ball.position.x + 300;

    ballGraphics.setPosition(ball.position.x, ball.position.y);
    ballGraphics.rotation = ball.angle;

    teeGraphics.setPosition(tee.position.x, tee.position.y);
    teeGraphics.rotation = tee.angle;

    clubGraphics.setPosition(club.position.x, club.position.y);
    clubGraphics.rotation = club.angle;

    lowerarmGraphics.setPosition(lowerarm.position.x + 3, lowerarm.position.y);
    lowerarmGraphics.rotation = lowerarm.angle;

    upperarmGraphics.setPosition(upperarm.position.x, upperarm.position.y);
    upperarmGraphics.rotation = upperarm.angle;

    ramenGraphics.setPosition(ramen.position.x, ramen.position.y);
    ramenGraphics.rotation = ramen.angle;

    retryText.setPosition(ball.position.x - 150, 250);

    if (Math.abs(pixelsToYards(ball.position.x)) > 5 && Math.abs(ball.velocity.x) <= 0.1 && ball.velocity.y <= 0.1 && ball.position.y - 540 <= 0.27) {
        if (!gameOver && pixelsToYards(ball.position.x) > max) {
            max = pixelsToYards(ball.position.x);
            maxGraphics.setPosition(yardsToPixels(max), 451);
            maxText.setPosition(yardsToPixels(max) + 7, 451 - 76);
            maxText.setText(max);
            best.setText(`Best: ${max}`);
        }

        retryText.visible = true;
        gameOver = true;
    }


    background.setPosition(ball.position.x, 500);
    background.tilePositionX = ball.position.x;
    background.tilePositionY = 0;
}

function pixelsToYards(pixels) {
    return Math.round((pixels - 400) / 20);
}

function yardsToPixels(yards) {
    return Math.round((yards * 20) + 400);
}

import { Joint, DOF } from '../../src/core/Joint.js';
import { Link } from '../../src/core/Link.js';
import { Goal } from '../../src/core/Goal.js';
import { Solver } from '../../src/core/Solver.js';

// planar arm of unit links rotating about Z with a position goal on the end
function createPlanarArm( count ) {

	const root = new Link();
	let current = root;
	const joints = [];
	for ( let i = 0; i < count; i ++ ) {

		const joint = new Joint();
		joint.setDoF( DOF.EZ );
		if ( i !== 0 ) joint.setPosition( 1, 0, 0 );
		joint.setDoFValues( 0.3 );
		current.addChild( joint );
		joints.push( joint );

		const link = new Link();
		joint.addChild( link );
		current = link;

	}

	const tip = new Joint();
	tip.setPosition( 1, 0, 0 );
	current.addChild( tip );
	const end = new Link();
	tip.addChild( end );

	const goal = new Goal();
	goal.setGoalDoF( DOF.X, DOF.Y, DOF.Z );
	goal.makeClosure( end );
	root.updateMatrixWorld( true );
	return { root, joints, goal, end };

}

function solve( root, goal, position, errorClamp = 0.1 ) {

	goal.setPosition( ...position );
	const solver = new Solver( root );
	solver.maxIterations = 200;
	solver.translationConvergeThreshold = 1e-7;
	solver.stallThreshold = 1e-9;
	solver.dampingFactor = 0.1;
	solver.translationErrorClamp = errorClamp;
	solver.solve();
	root.updateMatrixWorld( true );
	return solver;

}

function distance( end, position ) {

	const world = new Float64Array( 3 );
	end.getWorldPosition( world );
	return Math.hypot( world[ 0 ] - position[ 0 ], world[ 1 ] - position[ 1 ], world[ 2 ] - position[ 2 ] );

}

describe( 'Joint degree of freedom weights and stiffness', () => {

	it( 'default to unit weights and no springs, and reset with the degrees of freedom', () => {

		const joint = new Joint();
		joint.setDoF( DOF.EZ );
		expect( [ ...joint.dofWeights ] ).toEqual( [ 1, 1, 1, 1, 1, 1 ] );
		expect( [ ...joint.dofStiffness ] ).toEqual( [ 0, 0, 0, 0, 0, 0 ] );

		joint.dofWeights[ DOF.EZ ] = 5;
		joint.dofStiffness[ DOF.EZ ] = 2;
		joint.setDoF( DOF.EX );
		expect( joint.dofWeights[ DOF.EZ ] ).toBe( 1 );
		expect( joint.dofStiffness[ DOF.EZ ] ).toBe( 0 );

	} );

	it( 'moves lighter degrees of freedom more while still reaching the goal', () => {

		const target = [ 1.2, 1.9, 0 ];
		const moved = weights => {

			const { root, joints, goal, end } = createPlanarArm( 3 );
			joints.forEach( ( joint, i ) => joint.dofWeights[ DOF.EZ ] = weights[ i ] );
			solve( root, goal, target );
			expect( distance( end, target ) ).toBeLessThan( 1e-4 );
			return joints.map( joint => Math.abs( joint.getDoFValue( DOF.EZ ) - 0.3 ) );

		};

		const light = moved( [ 1, 1, 1e-3 ] );
		const heavy = moved( [ 1, 1, 1e3 ] );
		expect( light[ 2 ] ).toBeGreaterThan( heavy[ 2 ] * 2 );

	} );

	it( 'settles where the goal pull balances the spring', () => {

		const { root, joints, goal, end } = createPlanarArm( 1 );
		const [ joint ] = joints;
		const stiffness = 0.5;
		joint.dofStiffness[ DOF.EZ ] = stiffness;
		joint.setRestPoseValues( 0 );

		// The springs balance the goal error as clamped by the solver, so lift the clamp to
		// compare with the unclamped minimum of |p( t ) - g|^2 + k t^2, p( t ) = ( cos t, sin t ).
		const target = [ 0, 1, 0 ];
		solve( root, goal, target, Infinity );

		const angle = joint.getDoFValue( DOF.EZ );
		const gradient = 2 * ( Math.cos( angle ) - target[ 0 ] ) * - Math.sin( angle ) + 2 * ( Math.sin( angle ) - target[ 1 ] ) * Math.cos( angle ) + 2 * stiffness * angle;
		expect( angle ).toBeGreaterThan( 0.1 );
		expect( angle ).toBeLessThan( Math.PI / 2 - 0.1 );
		expect( Math.abs( gradient ) ).toBeLessThan( 1e-3 );
		expect( distance( end, target ) ).toBeGreaterThan( 0.05 );

	} );

	it( 'rejects weights that are not positive', () => {

		const { root, joints, goal } = createPlanarArm( 2 );
		joints[ 0 ].dofWeights[ DOF.EZ ] = 0;
		expect( () => solve( root, goal, [ 0.5, 1.5, 0 ] ) ).toThrow( /weights must be positive/ );

	} );

} );
